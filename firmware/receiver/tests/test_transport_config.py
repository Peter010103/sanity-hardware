import ctypes as c
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from sanity_zenoh.protocol import CTBR, command_packet, neutral
from sanity_zenoh.transport_test import TEST_CONFIG, decode_diagnostic


class TransportConfigTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();root=Path(__file__).resolve().parents[1];output=Path(cls.temp.name)/'test.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=undefined','-fno-sanitize-recover=all','-shared','-fPIC','-DCONFIG_SANITY_COMMS_TEST_ONLY=1','-I',str(root/'main'),str(root/'main/control.c'),str(root/'main/bench.c'),str(root/'main/test_config.c'),str(root/'tests/wrapper.c'),'-o',str(output)],check=True)
        cls.lib=c.CDLL(str(output));cls.lib.sz_test_config.argtypes=[c.c_char_p,c.c_size_t,c.c_uint8,c.c_uint32,c.c_bool,c.c_bool,c.c_bool,c.c_void_p]
        cls.lib.test_reset.argtypes=[c.c_uint,c.c_ulonglong];cls.lib.test_receive.argtypes=[c.c_char_p,c.c_uint,c.c_uint]


    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()


    def test_telemetry_toggle_requires_disabled_owner_and_echoes_nonce(self):
        for requested in (False,True):
            packet=TEST_CONFIG.pack(b'SZTM',1,1,requested,0,9,123);out=c.create_string_buffer(16)
            accepted=self.lib.sz_test_config(packet,len(packet),1,9,True,False,not requested,out)
            self.assertEqual(accepted,1)
            ack=decode_diagnostic(out.raw,'test_config',1)
            self.assertEqual(ack,{'enabled':requested,'code':0,'session':9,'nonce':123})
            for owner,valid,enabled in ((8,True,False),(9,False,False),(9,True,True)):
                self.assertEqual(self.lib.sz_test_config(packet,len(packet),1,owner,valid,enabled,not requested,out),0)
                ack=decode_diagnostic(out.raw,'test_config',1)
                self.assertEqual(ack['code'],1);self.assertEqual(ack['enabled'],not requested)


    def test_invalid_config_is_ignored_without_changing_reply(self):
        valid=TEST_CONFIG.pack(b'SZTM',1,1,1,0,9,123)
        for packet in [valid[:i] for i in range(16)]+[valid+b'x']+[valid[:i]+bytes([255])+valid[i+1:] for i in range(8)]:
            out=c.create_string_buffer(b'x'*16,16)
            self.assertEqual(self.lib.sz_test_config(packet,len(packet),1,9,True,False,False,out),-1)
            self.assertEqual(out.raw,b'x'*16)


    def test_test_firmware_rejects_enabled_zero_and_positive_commands(self):
        self.lib.test_reset(CTBR,123)
        packet=command_packet(1,CTBR,False,123,9,1,1000,1000,neutral(CTBR))
        self.assertTrue(self.lib.test_receive(packet,len(packet),1000))
        for seq,values in ((2,neutral(CTBR)),(3,(1500,1500,1500,1330))):
            packet=command_packet(1,CTBR,True,123,9,seq,1000,1000,values)
            self.assertFalse(self.lib.test_receive(packet,len(packet),1001))
        raw=(c.c_ubyte*48)();self.lib.test_status(1001,raw)
        from sanity_zenoh.protocol import ReceiverStatus
        self.assertFalse(ReceiverStatus.decode(bytes(raw)).enabled)


    def test_unordered_duplicate_sequences_rejected_and_one_second_lease_retained(self):
        self.lib.test_reset(CTBR,123)
        for seq,accepted in ((10,True),(9,False),(10,False),(11,True)):
            packet=command_packet(1,CTBR,False,123,9,seq,1000,1000,neutral(CTBR))
            self.assertEqual(bool(self.lib.test_receive(packet,len(packet),1000)),accepted)
        values=(c.c_ushort*4)()
        self.assertFalse(self.lib.test_output(1999,values));self.assertTrue(self.lib.test_output(2000,values))


if __name__=='__main__':unittest.main()
