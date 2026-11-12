import ctypes as c
from pathlib import Path
import struct
import sys
import unittest
R=next(p for p in Path(__file__).resolve().parents if (p/'firmware/receiver').is_dir())
sys.path.insert(0,str(R/'firmware/receiver/tests'))
from test_control import ControlTests
from sanity_zenoh.protocol import CTBR,ReceiverStatus,ctbr_si_values,si_command_packet

class SelectedFlipTests(ControlTests):
    @classmethod
    def setUpClass(cls):
        super().setUpClass();cls.lib.test_decode_values.argtypes=[c.c_char_p,c.c_uint,c.POINTER(c.c_ushort)]
    def packet(self,enabled=False,request=False,rates=(0.,0.,0.),seq=0,thrust=None):
        return si_command_packet(1,enabled,123,9,seq,1000,100,ctbr_si_values(enabled,(.7 if enabled else 0.) if thrust is None else thrust,rates,True,True),0xf04fc735,True,True,request)
    def test_actual_c_decoder_all_signs_limits_quantization_and_aux(self):
        self.assertTrue(self.lib.test_receive(self.packet(),40,1000))
        self.assertTrue(self.lib.test_receive(self.packet(True,seq=1,thrust=0.),40,1000))
        for index,rates in enumerate(((9.5,9.5,9.5),(-9.5,-9.5,-9.5),(0.,9.235,0.)),2):
            packet=self.packet(True,True,rates,index);values=(c.c_ushort*4)()
            self.assertEqual(packet[:5],b'SZC6\x06');self.assertTrue(self.lib.test_decode_values(packet,40,values))
            for axis in range(3):
                recovered=(values[axis]-1500)/500*600*3.141592653589793/180*(-1 if axis==2 else 1)
                self.assertLessEqual(abs(recovered-rates[axis]),.0105)
            self.assertTrue(self.lib.test_receive(packet,40,1000))
            raw=(c.c_ubyte*22)();self.lib.test_output_msp(1000,raw)
            self.assertEqual(struct.unpack('<8H',bytes(raw)[5:-1])[4:7],(2000,2000,2000))
    def test_session_bound_disabled_ack_then_expiry_clears_indi(self):
        packet=self.packet();self.assertTrue(self.lib.test_receive(packet,40,1000))
        raw=(c.c_ubyte*48)();self.lib.test_status(1000,raw);status=ReceiverStatus.decode(bytes(raw))
        self.assertTrue(status.indi_pitch_loop_supported);self.assertEqual(status.session,9);self.assertFalse(status.enabled)
        self.assertTrue(self.lib.test_receive(self.packet(True,seq=1,thrust=0.),40,1000))
        self.assertTrue(self.lib.test_receive(self.packet(True,True,(0.,9.5,0.),2),40,1000))
        out=(c.c_ubyte*22)();self.lib.test_output_msp(1100,out)
        self.assertEqual(struct.unpack('<8H',bytes(out)[5:-1])[4:7],(1000,2000,1000))
    def test_szc4_cannot_exceed_ordinary_pitch_limit(self):
        packet=bytearray(self.packet(True,True,(0.,9.5,0.)));packet[:5]=b'SZC4\x04'
        # At1e-4 scale9.5 cannot fit int16; ordinary v4 max23000 unchanged.
        struct.pack_into('>h',packet,34,23001)
        self.assertFalse(self.lib.test_receive(bytes(packet),40,1000))
    def test_szc3_still_rejects_high_roll_and_szc6_mapping_supports_it(self):
        with self.assertRaises(ValueError):ctbr_si_values(True,.7,(9.5,0.,0.),True)
        from sanity_zenoh.firmware_collective import FirmwareCollective
        mapping=FirmwareCollective(R/'releases/high-speed/host/sanity_zenoh/config/sanity15_indi_collective.json',high_rate=True,selected_high_rate=True)
        mapping.set_voltage(7.8)
        rc=mapping.convert(15,True,.7,(9.5,-9.5,9.5))
        self.assertEqual((rc.roll,rc.pitch,rc.yaw),(1954,1046,1046))

    def test_malformed_units_flags_and_limits_rejected(self):
        for offset,value in ((7,2),(7,4),(4,7)):
            packet=bytearray(self.packet());packet[offset]=value
            self.assertFalse(self.lib.test_receive(bytes(packet),40,1000))
        for axis,value in ((0,9501),(1,9501),(2,9501)):
            packet=bytearray(self.packet(True,True));struct.pack_into('>h',packet,32+2*axis,value)
            self.assertFalse(self.lib.test_receive(bytes(packet),40,1000))
        with self.assertRaises(ValueError):self.packet(False,True)

if __name__=='__main__':unittest.main()
