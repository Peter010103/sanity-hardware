import ctypes as c
from pathlib import Path
import subprocess,tempfile,unittest
from test_timing_wire import Event

class Window(c.Structure):
    _fields_=[(n,c.c_uint64) for n in ('epoch','start_us','previous_us','max_gap_us','max_at_us')]+[(n,c.c_uint32) for n in ('session','sequence','accepted','rejected','over_50ms')]+[('mode',c.c_uint8)]

class ArrivalWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();main=Path(__file__).resolve().parents[1]/'main';out=Path(cls.temp.name)/'window.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-pedantic','-shared','-fPIC',str(main/'timing_wire.c'),'-o',str(out)],check=True)
        cls.lib=c.CDLL(str(out));cls.lib.sz_timing_accept.argtypes=[c.POINTER(Window),c.c_uint64,c.c_uint32,c.c_uint32,c.c_uint8,c.c_bool,c.c_uint64];cls.lib.sz_timing_report.argtypes=[c.POINTER(Window),c.c_uint64,c.c_uint8,c.POINTER(Event)];cls.lib.sz_timing_report.restype=c.c_bool
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def test_gaps_are_measured_at_callback_and_survive_report_boundary(self):
        w=Window();event=Event()
        for seq,us in enumerate((1000,11000,21000)) :self.lib.sz_timing_accept(c.byref(w),123,9,seq,1,True,us)
        self.assertFalse(self.lib.sz_timing_report(c.byref(w),200000,1,c.byref(event)))
        self.assertTrue(self.lib.sz_timing_report(c.byref(w),201000,1,c.byref(event)))
        self.assertEqual((event.type,event.tick,event.t2,event.arg1),(8,3,10000,0))
        self.lib.sz_timing_accept(c.byref(w),123,9,3,1,True,230000)
        self.assertTrue(self.lib.sz_timing_report(c.byref(w),401000,1,c.byref(event)))
        self.assertEqual((event.tick,event.t2,event.t3,event.arg1),(1,209000,230000,1))
    def test_new_owner_or_epoch_does_not_mix_old_gaps(self):
        w=Window();event=Event()
        self.lib.sz_timing_accept(c.byref(w),123,9,1,1,True,1000)
        self.lib.sz_timing_accept(c.byref(w),124,10,1,1,True,1000000)
        self.lib.sz_timing_accept(c.byref(w),124,10,2,1,False,1010000)
        self.assertTrue(self.lib.sz_timing_report(c.byref(w),1200000,2,c.byref(event)))
        self.assertEqual((event.epoch,event.session,event.tick,event.t2,event.arg0,event.code),(124,10,1,0,1,2))
    def test_silence_report_cannot_invent_arrivals(self):
        w=Window();event=Event();self.assertFalse(self.lib.sz_timing_report(c.byref(w),500000,1,c.byref(event)))
        self.lib.sz_timing_accept(c.byref(w),123,9,1,1,True,1000)
        self.lib.sz_timing_report(c.byref(w),201000,1,c.byref(event))
        self.assertTrue(self.lib.sz_timing_report(c.byref(w),401000,1,c.byref(event)))
        self.assertEqual((event.tick,event.t2,event.t3),(0,0,0))

class ProfileTests(unittest.TestCase):
    def test_full_minimal_and_control_only_budgets(self):
        main=Path(__file__).resolve().parents[1]/'main'
        with tempfile.TemporaryDirectory() as folder:
            for level in (0,1,2):
                p=Path(folder)/f'profile{level}.so';subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-shared','-fPIC',f'-DCONFIG_SANITY_DIAGNOSTIC_LEVEL={level}',str(main/'telemetry_profile.c'),'-o',str(p)],check=True)
                lib=c.CDLL(str(p));lib.sz_telemetry_allow.argtypes=[c.c_void_p,c.c_uint8,c.c_uint8,c.c_uint64];lib.sz_telemetry_allow.restype=c.c_bool
                state=(c.c_uint64*512)();allowed=sum(lib.sz_telemetry_allow(state,2,102,t) for t in range(0,1000000,10000))
                self.assertEqual(allowed,(100,10,0)[level]);self.assertTrue(lib.sz_telemetry_allow(state,5,0,1000000))
                # Sampling telemetry cannot affect a separate command/UART path.
                self.assertEqual(sum(lib.sz_telemetry_allow(state,1,200,t) for t in range(0,1000000,10000)),(100,5,0)[level])

if __name__=='__main__':unittest.main()
