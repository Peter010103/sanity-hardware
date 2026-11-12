"""Native production HX711 tasks + fake HAL: no serial, network, flash or aircraft."""
import ctypes as c
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PROJECT=Path(__file__).resolve().parents[1]
ROOT=PROJECT.parents[3]
sys.path.insert(0,str(ROOT/'Sanity/sanity-hardware/tools/acquisition'))
from calibrate_onboard_tension import decode_packet


class State(c.Structure):
    _fields_=[('sequence',c.c_uint32),('resets',c.c_uint32),('last_packet_us',c.c_int64),
              ('settling_until_us',c.c_int64),('reset_us',c.c_int64),('discard',c.c_uint)]
class Sample(c.Structure):
    _fields_=[('sequence',c.c_uint32),('resets',c.c_uint32),('flags',c.c_uint32),
              ('max_high_us',c.c_uint32),('raw',c.c_int32),('ready_us',c.c_int64),
              ('ready_lower_us',c.c_int64),('shift_end_us',c.c_int64),('acquired_us',c.c_int64),('reset_us',c.c_int64)]


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();lib=Path(cls.temp.name)/'core.so'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-shared','-fPIC',str(PROJECT/'main/hx711_core.c'),'-o',str(lib)],check=True)
        cls.lib=c.CDLL(str(lib));cls.lib.sz_hx_init.argtypes=[c.POINTER(State),c.c_int64]
        cls.lib.sz_hx_step.argtypes=[c.POINTER(State),c.c_bool,c.c_int32,c.c_int64,c.c_int64,c.c_int64,c.c_uint32,c.c_bool,c.c_int64,c.POINTER(Sample),c.POINTER(c.c_bool)]
        cls.lib.sz_hx_step.restype=c.c_bool
        cls.lib.sz_hx_format.argtypes=[c.c_void_p,c.c_size_t,c.POINTER(Sample),c.c_uint,c.c_uint64,c.c_uint64,c.c_uint32,c.c_int64,c.c_uint32,c.c_uint32]
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def setUp(self):self.state=State();self.lib.sz_hx_init(c.byref(self.state),1)
    def step(self,now,have=True,raw=-500,high=3,stuck=False):
        sample=Sample();reset=c.c_bool()
        report=self.lib.sz_hx_step(c.byref(self.state),have,raw,now-100 if have else 0,now-500 if have else 0,now-10 if have else 0,high,stuck,now,c.byref(sample),c.byref(reset))
        return report,sample,reset.value
    def test_elapsed_settling_and_four_post_reset_discards(self):
        for now in (1000001,1100001,1200001,1300001):
            _,s,_=self.step(now);self.assertEqual(s.flags,8)
        _,s,_=self.step(1400001);self.assertEqual(s.flags,0);self.assertEqual(s.raw,-500)
        _,fault,reset=self.step(1500001,high=46)
        self.assertTrue(reset);self.assertEqual(fault.flags,12);self.assertEqual(fault.resets,2)
        self.assertEqual(self.state.discard,4)
        for now in (1510001,1520001,1530001,1540001,2000001):self.assertEqual(self.step(now)[1].flags,8)
        self.assertEqual(self.step(2100001)[1].flags,0)
    def test_missing_status_and_signed_saturation(self):
        self.assertFalse(self.step(999999,have=False)[0]);yes,s,_=self.step(1000000,have=False)
        self.assertTrue(yes);self.assertEqual((s.flags,s.ready_us,s.raw),(1,0,0))
        for raw in (-8388608,8388607):self.assertTrue(self.step(2000000,raw=raw)[1].flags&2)
    def test_sequence_and_reset_modulo_wrap(self):
        self.state.sequence=0xffffffff;self.state.resets=0xffffffff
        _,sample,reset=self.step(1000000,stuck=True)
        self.assertEqual(sample.sequence,0xffffffff);self.assertEqual(sample.resets,0);self.assertTrue(reset)
        self.assertEqual(self.step(1100000)[1].sequence,0)
    def test_legacy_fields_and_extension_fit_datagram(self):
        _,s,_=self.step(1000000)
        buf=c.create_string_buffer(768)
        size=self.lib.sz_hx_format(buf,len(buf),c.byref(s),9,0xffffffffffffffff,0xabcdef0123456789,0xffffffff,1000001,0xffffffff,0xffffffff)
        self.assertLess(size,768);packet=decode_packet(buf.value,9)
        self.assertEqual(packet['raw'],-500);self.assertEqual(packet['receiver_boot_id'],'abcdef0123456789')
        self.assertEqual(packet['schema'],'sanity_hx711_v1');self.assertEqual(packet['timing_v'],1)
        tiny=c.create_string_buffer(10);self.assertGreater(self.lib.sz_hx_format(tiny,10,c.byref(s),9,1,2,1,1000001,0,0),10)


class TaskTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=tempfile.TemporaryDirectory();cls.binary=Path(cls.temp.name)/'hx711_native'
        subprocess.run(['cc','-std=c11','-D_POSIX_C_SOURCE=200809L','-O2','-Wall','-Wextra','-Werror','-Wno-misleading-indentation','-pthread','-I',str(PROJECT/'tests/hx711_fakes'),'-I',str(PROJECT/'main'),str(PROJECT/'tests/hx711_native.c'),str(PROJECT/'main/hx711_core.c'),str(PROJECT/'main/control.c'),'-lm','-o',str(cls.binary)],check=True)
    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def run_mode(self,mode):
        r=subprocess.run([str(self.binary),mode],capture_output=True,text=True,check=True,timeout=8)
        if os.environ.get('HX711_NATIVE_EVIDENCE_DIR'):
            out=Path(os.environ['HX711_NATIVE_EVIDENCE_DIR']);out.mkdir(exist_ok=True,parents=True)
            (out/(mode+'.jsonl')).write_text(r.stdout);(out/(mode+'.stderr')).write_text(r.stderr)
        rows=[json.loads(line) for line in r.stdout.splitlines()];summary=rows.pop()
        self.assertGreater(summary['control_ticks'],200)
        for p in rows:decode_packet(json.dumps(p),9)
        return rows,summary
    def test_normal_settling_and_signed_raw(self):
        rows,s=self.run_mode('normal');self.assertGreater(s['reads'],50)
        self.assertTrue(any(p['flags']==8 for p in rows));self.assertTrue(any(p['flags']==0 for p in rows))
        self.assertTrue(all(p['raw']==-500 for p in rows))
        self.assertEqual(len({p['receiver_boot_id'] for p in rows}),1)
        for p in rows:
            if p['acquired_us']-p['reset_us']<600000:self.assertNotEqual(p['flags'],0)
    def test_missing_and_stuck_are_never_healthy(self):
        for mode in ('missing','stuck','gpio_fail'):
            with self.subTest(mode=mode):
                rows,s=self.run_mode(mode);self.assertTrue(rows);self.assertTrue(all(p['flags'] for p in rows))
                if mode=='missing':self.assertTrue(all(p['flags']&1 for p in rows));self.assertEqual(s['reads'],0)
                if mode=='stuck':self.assertTrue(all(p['flags']&4 for p in rows))
    def test_saturation_is_never_healthy(self):
        rows,_=self.run_mode('saturation');self.assertTrue(all(p['flags']&2 for p in rows))
    def test_clock_reset_and_clean_recovery(self):
        rows,_=self.run_mode('clock_fault');self.assertTrue(any(p['flags']&4 for p in rows))
        self.assertEqual(max(p['resets'] for p in rows),2)
        for p in rows:
            if p['resets']==2 and p['acquired_us']-p['reset_us']<600000:self.assertTrue(p['flags'])
        self.assertTrue(any(p['resets']==2 and p['flags']==0 for p in rows))
    def test_network_failures_do_not_stop_acquisition_or_control(self):
        for mode in ('unreachable','congested','socket_fail','blocked'):
            with self.subTest(mode=mode):
                rows,s=self.run_mode(mode);self.assertGreater(s['reads'],50)
                if mode=='unreachable':self.assertEqual(s['sends'],0)
                elif mode=='blocked':self.assertTrue(any(p['queue_drops']>0 for p in rows))
                else:self.assertTrue(any(p['send_drops']>0 for p in rows))
    def test_optional_allocation_failures_do_not_abort_receiver(self):
        for mode in ('queue_fail','read_task_fail','udp_task_fail'):
            with self.subTest(mode=mode):rows,_=self.run_mode(mode);self.assertFalse(rows)
