import ctypes as c
import random
import struct
import unittest
from test_si_control import PhysicalControlTests
from sanity_zenoh.firmware_collective import FirmwareCollective
from sanity_zenoh.protocol import ReceiverStatus,ctbr_si_values,si_command_packet


class PitchWireTests(PhysicalControlTests):
    def test_high_pitch_C_host_roundtrip_and_legacy_boundary(self):
        from test_si_control import PROFILE
        mapping=FirmwareCollective(PROFILE,high_rate=True);mapping.set_voltage(7.8)
        rng=random.Random(99)
        for pitch in [-9.5,-8.93,-3.3,0.,3.3,8.93,9.5]+[rng.uniform(-9.5,9.5) for _ in range(1000)]:
            rates=(rng.uniform(-2.3,2.3),pitch,rng.uniform(-2.6,2.6))
            values=ctbr_si_values(True,.6815,rates,True)
            packet=si_command_packet(1,True,123,9,2,1000,100,values,mapping.calibration_id,True)
            self.assertEqual(packet[:5],b'SZC3\x03')
            ok,actual=self.decode(packet);expected=mapping.convert(1,True,.6815,rates)
            self.assertTrue(ok);self.assertEqual(actual,(expected.roll,expected.pitch,expected.yaw,expected.throttle))
        with self.assertRaises(ValueError):ctbr_si_values(True,.5,(0.,9.,0.))
        with self.assertRaises(ValueError):ctbr_si_values(True,.5,(0.,9.501,0.),True)
        with self.assertRaises(ValueError):ctbr_si_values(True,.5,(2.301,0.,0.),True)

    def test_capability_ack_zero_enable_and_expiry(self):
        calibration=self.mapping.calibration_id
        def packet(seq,enabled=False,rates=(0.,0.,0.),thrust=0.):
            return si_command_packet(1,enabled,123,9,seq,1000,100,ctbr_si_values(enabled,thrust,rates,True),calibration,True)
        p=packet(0);self.assertTrue(self.lib.test_receive(p,len(p),1000))
        raw=(c.c_ubyte*48)();self.lib.test_status(1001,raw)
        self.assertTrue(ReceiverStatus.decode(bytes(raw)).pitch_loop_supported)
        p=packet(1,True,(0.,9.,0.),.5);self.assertFalse(self.lib.test_receive(p,len(p),1001))
        p=packet(1,True);self.assertTrue(self.lib.test_receive(p,len(p),1001))
        p=packet(2,True,(0.,9.,0.),.5);self.assertTrue(self.lib.test_receive(p,len(p),1002))
        self.assertGreater(self.output(1003)[1][1],1900)
        self.assertTrue(self.output(1100)[0]);self.assertEqual(self.output(1100)[1],(1500,1500,1500,1000))
        p=self.packet(seq=3);self.assertTrue(self.lib.test_receive(p,len(p),1101))
        self.lib.test_status(1102,raw);self.assertFalse(ReceiverStatus.decode(bytes(raw)).pitch_loop_supported)

    def test_high_rate_malformed_and_wrong_version_rejected(self):
        values=ctbr_si_values(True,.5,(0.,9.5,0.),True)
        p=bytearray(si_command_packet(1,True,123,9,1,1000,100,values,self.mapping.calibration_id,True))
        struct.pack_into('>h',p,34,9501);self.assertFalse(self.decode(bytes(p))[0])
        struct.pack_into('>h',p,34,9500);p[4]=2;self.assertFalse(self.decode(bytes(p))[0])


    def test_one_second_pitch_lease_survives_short_gap_then_latches_stop(self):
        calibration=self.mapping.calibration_id
        def packet(seq,enabled=False,lease=1000,thrust=0.):
            return si_command_packet(1,enabled,123,9,seq,lease,1000,
                ctbr_si_values(enabled,thrust,(0.,0.,0.),True),calibration,True)
        for seq,enabled,when,thrust in [(0,False,1000,0.),(1,True,1001,0.),(2,True,1002,.5)]:
            p=packet(seq,enabled,thrust=thrust)
            self.assertTrue(self.lib.test_receive(p,len(p),when))
        self.assertFalse(self.output(1150)[0])
        self.assertGreater(self.output(1150)[1][3],1000)
        p=packet(3,True,lease=1150,thrust=.5)
        self.assertTrue(self.lib.test_receive(p,len(p),1250))
        self.assertFalse(self.output(2149)[0])
        self.assertTrue(self.output(2150)[0])
        self.assertEqual(self.output(2150)[1],(1500,1500,1500,1000))
        p=packet(4,True,lease=2150,thrust=.5)
        self.assertFalse(self.lib.test_receive(p,len(p),2151))
        p=packet(4,False,lease=2150)
        self.assertTrue(self.lib.test_receive(p,len(p),2152))
        p=packet(5,True,lease=2150)
        self.assertTrue(self.lib.test_receive(p,len(p),2153))
        p=packet(6,True,lease=2150,thrust=.5)
        self.assertTrue(self.lib.test_receive(p,len(p),2154))
