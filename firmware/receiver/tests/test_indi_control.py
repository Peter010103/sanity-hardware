"""Exercise real ESP parser, authority lease and emitted FC AUX channel."""
import ctypes as c
import struct
from test_control import ControlTests
from sanity_zenoh.protocol import si_command_packet, ctbr_si_values

class IndiControlTests(ControlTests):
    def packet(self, seq=0, enabled=False, request=False, thrust=0):
        return si_command_packet(1,enabled,123,9,seq,1000,100,
            ctbr_si_values(enabled,thrust,(0,0,0)),0xf04fc735,
            controller_selection=True,indi_requested=request)
    def accept(self, **kwargs):
        p=self.packet(**kwargs)
        return bool(self.lib.test_receive(p,len(p),1000))
    def channels(self, now=1000):
        out=(c.c_ubyte*22)();self.lib.test_output_msp(now,out)
        return struct.unpack('<8H',bytes(out)[5:-1])
    def test_selection_is_distinct_from_arm_and_cleared_by_disable_or_expiry(self):
        self.assertTrue(self.accept())
        self.assertTrue(self.accept(seq=1,enabled=True))
        self.assertEqual(self.channels()[4:7],(2000,2000,1000))
        self.assertTrue(self.accept(seq=2,enabled=True,request=True,thrust=.68))
        self.assertEqual(self.channels()[4:7],(2000,2000,2000))
        self.assertEqual(self.channels(1100)[4:7],(1000,2000,1000))
        self.assertFalse(self.accept(seq=3,enabled=True,request=True))
        self.assertTrue(self.accept(seq=3))
        self.assertEqual(self.channels()[4:7],(1000,2000,1000))
    def test_bad_flags_and_disabled_request_rejected_by_c(self):
        for flags in (2,4,5,7,255):
            p=bytearray(self.packet());p[7]=flags
            self.assertFalse(self.lib.test_receive(bytes(p),len(p),1000))
        self.assertFalse(self.accept(enabled=True,request=True,thrust=.68))
    def test_disabled_host_cannot_request_indi(self):
        with self.assertRaises(ValueError):self.packet(request=True)
