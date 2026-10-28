import copy
import json
import unittest
from calibrate_onboard_tension import decode_packet
from hx711_timing import HX711Health


def packet(seq=1,ready=1000000,**extra):
    value=dict(schema='sanity_hx711_v1',device_id=9,boot_id='1234567890abcdef',seq=seq,
        ready_us=ready,read_us=ready+140,raw=-500,flags=0,max_high_us=3,send_drops=0,
        resets=1,dout_gpio=1,sck_gpio=2,channel='A',gain=128,timing_v=1,
        receiver_boot_id='abcdef0123456789',receiver_boot_count=17,
        ready_lower_us=ready-500,shift_end_us=ready+100,acquired_us=ready+120,
        reset_us=1000,queue_drops=0)
    value.update(extra);return value


class TimingTests(unittest.TestCase):
    def test_additive_v1_and_old_packet_compatibility(self):
        old={k:v for k,v in packet().items() if k not in ('timing_v','receiver_boot_id','receiver_boot_count','ready_lower_us','shift_end_us','acquired_us','reset_us','queue_drops')}
        self.assertEqual(decode_packet(json.dumps(old),9)['raw'],-500)
        self.assertEqual(decode_packet(json.dumps(packet()),9)['raw'],-500)
    def test_inconsistent_metadata_and_unknown_flags_rejected(self):
        for change in ({'timing_v':2},{'flags':16},{'receiver_boot_id':None},{'acquired_us':1000200},
                       {'shift_end_us':999999},{'ready_lower_us':1000001},{'queue_drops':True}):
            with self.subTest(change=change),self.assertRaises((ValueError,TypeError)):decode_packet(json.dumps(packet(**change)),9)
        p=packet();del p['timing_v']
        with self.assertRaises(ValueError):decode_packet(json.dumps(p),9)
    def test_measured_non_nominal_intervals_and_64bit_clock(self):
        h=HX711Health();a=packet(0xffffffff,ready=(1<<32)*1000+1000000)
        h.observe(a,1000000000);b=packet(0,ready=a['ready_us']+11073)
        r=h.observe(b,1011073000);self.assertTrue(r['sample_usable']);self.assertEqual(r['source_interval_us'],11073)
        self.assertFalse(r['clock_alignment_verified'])
    def test_loss_reordering_duplicates_do_not_refresh_watchdog(self):
        h=HX711Health();h.observe(packet(1),1000000000)
        loss=h.observe(packet(3,1011073),1011073000);self.assertFalse(loss['sample_usable']);self.assertEqual(loss['sequence_gap'],1)
        for seq in (3,2):self.assertFalse(h.observe(packet(seq),1020000000)['sample_usable'])
        self.assertTrue(h.check(1300000000)['stale'])
        self.assertTrue(h.observe(packet(4,1022146),1030000000)['sample_usable'])
    def test_counter_change_invalidates_baseline_even_if_fault_packet_lost(self):
        h=HX711Health();first=h.observe(packet(),1000000000)
        reset=h.observe(packet(2,1011073,resets=2,reset_us=1005000),1011073000)
        self.assertTrue(reset['baseline_invalidated']);self.assertFalse(reset['sample_usable'])
        self.assertGreater(reset['baseline_generation'],first['baseline_generation'])
        self.assertFalse(h.check(1012000000)['health_ok'])
    def test_boot_change_rejects_delayed_previous_boot(self):
        h=HX711Health();old=packet();h.observe(old,1000000000)
        new=packet(0,20000,boot_id='0000000000000001',reset_us=1000)
        r=h.observe(new,1020000000);self.assertTrue(r['baseline_invalidated']);self.assertIn('boot_changed',r['reasons'])
        old['seq']=2;self.assertEqual(h.observe(old,1030000000)['reasons'],['retired_boot'])
    def test_all_fault_flags_and_stale_local_queue_are_unusable(self):
        for flag in (1,2,4,8):
            p=packet(flags=flag)
            if flag==1:p.update(ready_us=0,ready_lower_us=0,shift_end_us=0,raw=0)
            if flag==2:p['raw']=8388607
            p=decode_packet(json.dumps(p),9)
            self.assertFalse(HX711Health().observe(p,1000000000)['sample_usable'])
        p=packet(read_us=1200000);self.assertIn('local_queue_age',HX711Health().observe(p,1000000000)['reasons'])
    def test_no_arrivals_timeout_and_send_loss(self):
        h=HX711Health();self.assertFalse(h.check(0)['health_ok']);h.observe(packet(),1000000000)
        r=h.observe(packet(2,1011073,send_drops=1),1011073000);self.assertIn('send_drops_changed',r['reasons'])
        self.assertFalse(h.check(1400000000)['health_ok'])
    def test_no_signal_drift_or_cable_decision_is_inferred(self):
        h=HX711Health();p=packet();h.observe(p,1000000000)
        for i in range(2,100):
            p=packet(i,1000000+(i-1)*11073,raw=-500-i*300)
            r=h.observe(p,1000000000+(i-1)*11073000)
            self.assertTrue(r['sample_usable']);self.assertNotIn('cable_state',r)
    def test_clock_discontinuities_and_long_source_interval(self):
        for ready,reason in ((900000,'source_clock_reversed'),(1200000,'source_gap')):
            h=HX711Health();h.observe(packet(),1000000000)
            r=h.observe(packet(2,ready),1020000000)
            self.assertFalse(r['sample_usable']);self.assertIn(reason,r['reasons'])
            self.assertFalse(h.check(1020000001)['health_ok'])
        h=HX711Health();h.observe(packet(),1000000000)
        self.assertIn('host_clock_reversed',h.observe(packet(2,1011073),999999999)['reasons'])
    def test_modular_reset_counter_wrap_invalidates_old_baseline(self):
        h=HX711Health();a=packet(resets=0xffffffff);h.observe(a,1000000000)
        b=packet(2,1011073,resets=0,reset_us=1005000)
        decode_packet(json.dumps(b),9);r=h.observe(b,1011073000)
        self.assertTrue(r['baseline_invalidated']);self.assertIn('converter_reset',r['reasons'])
    def test_decoder_failure_invalidates_health_without_refreshing_freshness(self):
        h=HX711Health();h.observe(packet(),1000000000)
        h.invalidate('malformed_packet')
        self.assertFalse(h.check(1010000000)['health_ok'])
        self.assertTrue(h.check(1300000000)['stale'])
        self.assertTrue(h.observe(packet(2,1011073),1011073000)['sample_usable'])
