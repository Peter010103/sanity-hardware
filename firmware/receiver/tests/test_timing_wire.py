"""Cross-language timing frames must preserve integer timestamps and identity."""
import ctypes as c
from pathlib import Path
import struct,subprocess,tempfile,unittest

class Event(c.Structure):
    _fields_=[(x,c.c_uint8) for x in ['type','mode','flags','code']]+[(x,c.c_uint32) for x in ['session','sequence','tick']]+[(x,c.c_uint64) for x in ['epoch','t0','t1','t2','t3']]+[(x,c.c_uint32) for x in ['arg0','arg1']]

class TimingWireTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();root=Path(__file__).resolve().parents[1]/'main';out=Path(cls.tmp.name)/'timing.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-shared','-fPIC',str(root/'timing_wire.c'),'-o',str(out)],check=True)
        cls.lib=c.CDLL(str(out));cls.lib.sz_probe_reply.argtypes=[c.c_char_p,c.c_size_t,c.c_uint8,c.c_uint64,c.c_uint64,c.c_uint64,c.c_int8,c.c_uint8,c.c_void_p];cls.lib.sz_probe_reply.restype=c.c_bool
        cls.lib.sz_trace_encode.argtypes=[c.POINTER(Event),c.c_void_p]
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_probe_endian_echo_and_integer_timestamps(self):
        packet=struct.pack('>4sBBHIQ',b'SZP1',1,1,0,42,9876543210123);out=c.create_string_buffer(48)
        self.assertTrue(self.lib.sz_probe_reply(packet,len(packet),1,123456,123499,9876543210,-57,11,out))
        self.assertEqual(struct.unpack('>4sBBHIQQQQbBBB',out.raw),(b'SZR1',1,1,0,42,9876543210123,123456,123499,9876543210,-57,11,0,0))
        for offset,value in [(0,0),(4,2),(5,2),(6,1),(7,1)]:
            bad=bytearray(packet);bad[offset]=value;self.assertFalse(self.lib.sz_probe_reply(bytes(bad),len(bad),1,1,2,3,-40,1,out))
        for n in range(len(packet)):self.assertFalse(self.lib.sz_probe_reply(packet[:n],n,1,1,2,3,-40,1,out))
    def test_trace_layout_matches_host(self):
        values=(3,2,1,214,123,456,789,2**50,1000,1001,1002,1003,14,0)
        out=c.create_string_buffer(64);self.lib.sz_trace_encode(c.byref(Event(*values)),out)
        self.assertEqual(struct.unpack('>4B3I5Q2I',out.raw),values)

if __name__=='__main__':unittest.main()
