"""Native receiver decision/clock tests and schema1 cross-language contract."""
import ctypes as c
from pathlib import Path
import struct,subprocess,tempfile,unittest
from sanity_zenoh.command_timing import decode_command_trace

KEYS=('type','mode','flags','code','session','sequence','tick','epoch','t0','t1','t2','t3','arg0','arg1')
class Command(c.Structure):
    _fields_=[('epoch',c.c_uint64)]+[(x,c.c_uint32) for x in ('session','sequence','lease_ms')]+[
        ('ttl_ms',c.c_uint16),('values',c.c_uint16*4),('thrust_q',c.c_uint16),('mode',c.c_uint8),('device',c.c_uint8)]+[
        (x,c.c_bool) for x in ('enabled','physical_units','high_rate','controller_selection','indi_requested','paper_native')]+[
        ('thrust_n',c.c_float),('alpha_rad_s2',c.c_float*3),('fallback_rate_rad_s',c.c_float*3)]
class State(c.Structure):
    _fields_=[('epoch',c.c_uint64),('command',Command),('last_rx_ms',c.c_uint32),('rejected',c.c_uint32),
        ('device',c.c_uint8),('mode',c.c_uint8),('valid',c.c_bool),('enabled',c.c_bool),('rearm_required',c.c_bool)]
class Trace(c.Structure):
    _fields_=[(x,c.c_uint8) for x in ('type','mode','flags','code')]+[(x,c.c_uint32) for x in ('session','sequence','tick')]+[
        (x,c.c_uint64) for x in ('epoch','t0','t1','t2','t3')]+[(x,c.c_uint32) for x in ('arg0','arg1')]

class CommandTimingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();root=Path(__file__).resolve().parents[1]/'main';out=Path(cls.tmp.name)/'timing.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-fsanitize=undefined',
            '-fno-sanitize-recover=all','-shared','-fPIC','-I',str(root),str(root/'timing_wire.c'),str(root/'control.c'),str(root.parent/'tests/control_layout.c'),'-o',str(out)],check=True)
        cls.lib=c.CDLL(str(out))
        cls.lib.sz_test_command_size.restype=c.c_size_t;cls.lib.sz_test_state_size.restype=c.c_size_t
        if cls.lib.sz_test_command_size()!=c.sizeof(Command) or cls.lib.sz_test_state_size()!=c.sizeof(State):
            raise RuntimeError('ctypes control layout differs from actual C ABI')
        cls.lib.sz_init.argtypes=[c.POINTER(State),c.c_uint8,c.c_uint8,c.c_uint64]
        cls.lib.sz_accept.argtypes=[c.POINTER(State),c.POINTER(Command),c.c_uint32];cls.lib.sz_accept.restype=c.c_bool
        cls.lib.sz_command_reject_reason.argtypes=[c.POINTER(State),c.POINTER(Command),c.c_bool,c.c_uint8,c.c_uint32];cls.lib.sz_command_reject_reason.restype=c.c_uint8
        cls.lib.sz_command_decision_trace.argtypes=[c.POINTER(Command),c.c_bool,c.c_bool,c.c_uint8,c.c_uint32,c.c_uint64,c.c_uint64,c.c_uint64,c.c_uint32];cls.lib.sz_command_decision_trace.restype=Trace
        cls.lib.sz_command_uart_trace.argtypes=[c.POINTER(State),c.c_uint32,c.c_uint8,c.c_uint64,c.c_uint64,c.c_uint64,c.c_uint64,c.c_uint,c.c_int];cls.lib.sz_command_uart_trace.restype=Trace
        cls.lib.sz_trace_encode.argtypes=[c.POINTER(Trace),c.c_void_p]
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def setUp(self):
        self.s=State();self.lib.sz_init(c.byref(self.s),15,1,2**50)
        self.cmd=Command(epoch=2**50,session=42,sequence=1,lease_ms=1000,ttl_ms=100,
            values=(c.c_uint16*4)(1500,1500,1500,1000),mode=1,device=15)
    def reason(self,cmd=None,now=1001,stage=0):
        cmd=cmd or self.cmd;before=State.from_buffer_copy(self.s)
        accepted=self.lib.sz_accept(c.byref(self.s),c.byref(cmd),now) if not stage else False
        return accepted,self.lib.sz_command_reject_reason(c.byref(before),c.byref(cmd),accepted,stage,now)
    def decode(self,event):
        out=c.create_string_buffer(64);self.lib.sz_trace_encode(c.byref(event),out)
        return decode_command_trace(dict(zip(KEYS,struct.unpack('>4B3I5Q2I',out.raw))))
    def test_accept_and_exact_identity_and_receiver_lease(self):
        self.assertEqual(self.reason(),(True,0))
        event=self.lib.sz_command_decision_trace(c.byref(self.cmd),True,True,0,1001,1000100,1000109,990000,7)
        r=self.decode(event);self.assertEqual(r['session'],42);self.assertEqual(r['epoch'],2**50)
        self.assertEqual(r['lease_age_ms'],1);self.assertEqual(r['decision_duration_us'],9)
        self.assertEqual(r['accepted_gap_us'],10100);self.assertEqual(r['rejected_counter'],7)
    def test_rejection_sequence_stale_epoch_device_rearm_and_stage(self):
        self.reason();self.assertEqual(self.reason(),(False,9))
        self.cmd.sequence=2;self.assertEqual(self.reason(now=1100),(False,8))
        self.cmd.epoch+=1;self.assertEqual(self.reason(),(False,5))
        self.cmd.epoch-=1;self.cmd.device=16;self.assertEqual(self.reason(),(False,6))
        self.cmd.device=15;self.cmd.enabled=True;self.s.rearm_required=True
        self.assertEqual(self.reason(),(False,11));self.assertEqual(self.reason(stage=4),(False,4))
    def test_wrap_and_uart_boundary(self):
        self.cmd.lease_ms=2**32-2
        self.assertEqual(self.reason(now=2),(True,0))
        before=(2**32+2)*1000+100
        e=self.lib.sz_command_uart_trace(c.byref(self.s),2**32-1,200,before-600,before-400,before,before+12,22,22)
        r=self.decode(e);self.assertEqual(r['lease_age_at_submit_ms'],4)
        self.assertEqual(r['received_to_submit_us'],600);self.assertEqual(r['uart_api_duration_us'],12)
        self.assertTrue(r['driver_full_write']);self.assertIsNone(r['fc_application_time_us'])
        e=self.lib.sz_command_uart_trace(c.byref(self.s),0,200,before-600,before-400,before,before+12,22,-1)
        r=self.decode(e);self.assertFalse(r['driver_full_write']);self.assertEqual(r['driver_returned_bytes'],-1)
    def test_future_lease_does_not_fake_latency(self):
        self.cmd.lease_ms=1002;self.assertEqual(self.reason(now=1001),(False,8))
        e=self.lib.sz_command_decision_trace(c.byref(self.cmd),True,False,8,1001,1001000,1001010,0,1)
        self.assertEqual(self.decode(e)['lease_age_ms'],2**32-1)

if __name__=='__main__':unittest.main()
