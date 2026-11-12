import ctypes as c
import json
from pathlib import Path
import random
import struct
import unittest

from test_control import ControlTests
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.protocol import CTBR, ReceiverStatus, ctbr_si_values, si_command_packet

ROOT=Path(__file__).resolve().parents[4]
PROFILE=ROOT/'src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_esp_collective.json'


class PhysicalControlTests(ControlTests):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.mapping=FirmwareCollective(PROFILE)
        cls.mapping.set_voltage(7.8)
        cls.lib.test_decode_values.argtypes=[c.c_char_p,c.c_uint,c.POINTER(c.c_ushort)]

    def setUp(self):
        super().setUp()
        self.mapping.set_voltage(7.8)

    def packet(self,seq=0,armed=False,thrust=0.,rates=(0.,0.,0.),calibration_id=None):
        return si_command_packet(1,armed,123,9,seq,1000,1000,ctbr_si_values(armed,thrust,rates),calibration_id or self.mapping.calibration_id)

    def decode(self,packet):
        values=(c.c_ushort*4)()
        result=self.lib.test_decode_values(packet,len(packet),values)
        return bool(result),tuple(values)

    def test_si_c_conversion_matches_shared_host_reference_across_domain(self):
        rng=random.Random(27)
        for thrust in [0.,.0001,.0384,.0385,.6815,2.663,3.2]+[rng.random()*3.2 for _ in range(3000)]:
            rates=tuple(rng.uniform(-limit,limit) for limit in (2.3,2.3,2.6))
            packet=self.packet(armed=True,thrust=thrust,rates=rates)
            ok,values=self.decode(packet)
            expected=self.mapping.convert(1,True,thrust,rates)
            self.assertTrue(ok)
            self.assertEqual(values,(expected.roll,expected.pitch,expected.yaw,expected.throttle))

    def test_si_disabled_zero_positive_and_status_output(self):
        packet=self.packet()
        self.assertTrue(self.lib.test_receive(packet,len(packet),1000))
        positive=self.packet(seq=1,armed=True,thrust=.6815)
        self.assertFalse(self.lib.test_receive(positive,len(positive),1001))
        zero=self.packet(seq=1,armed=True)
        self.assertTrue(self.lib.test_receive(zero,len(zero),1001))
        self.assertEqual(self.output(1001),(False,(1500,1500,1500,1000)))
        positive=self.packet(seq=2,armed=True,thrust=.6815,rates=(.5,-.5,1.))
        self.assertTrue(self.lib.test_receive(positive,len(positive),1002))
        raw=(c.c_ubyte*48)();self.lib.test_status(1003,raw)
        status=ReceiverStatus.decode(bytes(raw))
        self.assertEqual(status.calibration_id,self.mapping.calibration_id)
        self.assertEqual(status.output_values,self.output(1003)[1])
        self.assertEqual(status.output_values[-1],self.mapping.convert(1,True,.6815,(.5,-.5,1.)).throttle)
        self.assertLess(status.output_values[2],1500)
        self.assertEqual(self.output(2000),(True,(1500,1500,1500,1000)))

    def test_si_wrong_calibration_units_and_malformed_inputs_rejected(self):
        bad=self.packet(armed=True,thrust=.5,calibration_id=self.mapping.calibration_id ^ 1)
        self.assertFalse(self.decode(bad)[0])
        for offset,value in [(4,1),(5,2),(7,2)]:
            packet=bytearray(self.packet());packet[offset]=value
            self.assertFalse(self.decode(bytes(packet))[0])
        for channel,value in [(0,23001),(1,-23001),(2,26001),(3,32001)]:
            packet=bytearray(self.packet(armed=True))
            struct.pack_into('>H',packet,32+channel*2,value & 65535)
            self.assertFalse(self.decode(bytes(packet))[0])
        packet=bytearray(self.packet());struct.pack_into('>H',packet,38,1)
        self.assertFalse(self.decode(bytes(packet))[0])
        self.assertFalse(self.decode(self.packet()[:-1])[0])

    def test_profile_integrity_is_checked(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            data=json.loads(PROFILE.read_text());data['speed_model']['coefficients_rad_s'][0]+=1
            path=Path(directory)/'profile.json';path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                FirmwareCollective(path)

    def test_voltage_aware_inverse_matches_C_across_measured_support(self):
        rng=random.Random(4)
        self.lib.test_set_voltage.argtypes=[c.c_uint]
        for mv in [0,6000,6600,6620,6800,7000,7200,7400,7600,7800,8000,8100,8170,8200,8400,9000]:
            self.lib.test_set_voltage(mv);self.mapping.set_voltage(mv/1000)
            rc,force=self.mapping.curve()
            for _ in range(200):
                thrust=rng.random()*3.2
                packet=self.packet(armed=True,thrust=thrust)
                ok,values=self.decode(packet)
                self.assertTrue(ok)
                self.assertEqual(values[-1],self.mapping.convert(1,True,thrust,(0.,0.,0.)).throttle)
                self.assertGreaterEqual(values[-1],rc[0]);self.assertLessEqual(values[-1],rc[-1])
        self.lib.test_set_voltage(0);self.mapping.set_voltage(0)
        fallback=self.decode(self.packet(armed=True,thrust=.6815))[1]
        self.lib.test_set_voltage(8000);self.mapping.set_voltage(8.)
        self.assertEqual(fallback,self.decode(self.packet(armed=True,thrust=.6815))[1])
        self.lib.test_set_voltage(7800);self.mapping.set_voltage(7.8)
