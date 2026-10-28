import unittest
from tension_pull_protocol import PullProtocol


class PullProtocolTests(unittest.TestCase):
    def test_no_cues_before_live_data(self):
        p=PullProtocol()
        self.assertEqual(p.update(1000),[])
        self.assertEqual(p.phase,'waiting_for_data')

    def test_default_instructions_and_holds(self):
        p=PullProtocol();self.assertEqual(p.duration_s,34)
        p.start(10)
        expected=[(10,'get_ready',0),(15,'baseline',0),(20,'pull',1),(23,'release',1),
                  (28,'pull',2),(31,'release',2),(36,'pull',3),(39,'release',3),(44,'complete',3)]
        for at,phase,cycle in expected:
            events=p.update(at);self.assertEqual(len(events),1)
            self.assertEqual((events[0]['phase'],events[0]['cycle']),(phase,cycle))
            if phase!='complete':self.assertEqual(p.update(at+.001),[])
        self.assertTrue(p.complete)
        self.assertEqual(p.update(100),[])

    def test_delayed_loop_never_skips_or_shortens_next_phase(self):
        p=PullProtocol(cycles=1,pull_s=3,release_s=5,baseline_s=1,countdown_s=0)
        p.start(0);self.assertEqual(p.update(0)[0]['phase'],'baseline')
        event=p.update(10)[0]
        self.assertEqual(event['phase'],'pull');self.assertEqual(event['late_by_s'],9)
        self.assertEqual(p.update(12.99),[])
        self.assertEqual(p.update(13)[0]['phase'],'release')
        self.assertEqual(p.update(17.99),[])
        self.assertEqual(p.update(18)[0]['phase'],'complete')

    def test_known_weight_repeats_and_removes_mass(self):
        p=PullProtocol(cycles=3,weights_g=[10.0])
        loads=[x for x in p.stages if x['phase']=='pull']
        releases=[x for x in p.stages if x['phase']=='release']
        self.assertEqual(len(loads),3)
        self.assertEqual([x['added_weight_g'] for x in loads],[10,10,10])
        self.assertTrue(all(x['added_weight_g']==0 and x['reference_force_N']==0 for x in releases))
        self.assertAlmostEqual(loads[0]['reference_force_N'],.0980665)
        self.assertIn('LOAD 10 g',loads[0]['instruction'])

    def test_weight_list_and_constant_preload_are_not_cumulative(self):
        p=PullProtocol(cycles=2,weights_g=[5.0,10.0],base_load_g=77.03)
        loads=[x for x in p.stages if x['phase']=='pull'];releases=[x for x in p.stages if x['phase']=='release']
        self.assertEqual(p.cycles,4)
        self.assertEqual([x['added_weight_g'] for x in loads],[5,10,5,10])
        self.assertEqual([x['repeat'] for x in loads],[1,1,2,2])
        self.assertAlmostEqual(loads[1]['reference_force_N'],.08703*9.80665)
        self.assertTrue(all(x['total_reference_mass_g']==77.03 for x in releases))

    def test_hand_pulls_do_not_claim_known_force(self):
        p=PullProtocol()
        self.assertTrue(all(x['reference_force_N'] is None for x in p.stages))

    def test_known_weights_wait_for_each_enter_and_start_timer_after_it(self):
        p=PullProtocol(cycles=1,weights_g=[10],countdown_s=0,baseline_s=1,pull_s=3,release_s=5)
        p.start(0)
        self.assertEqual(p.update(0)[0]['phase'],'setup_baseline')
        self.assertEqual(p.update(100),[])
        self.assertIsNone(p.current['reference_force_N'])
        self.assertEqual(p.confirm(100)[0]['phase'],'baseline')
        self.assertEqual(p.update(100.99),[])
        self.assertEqual(p.update(101)[0]['phase'],'setup_pull')
        self.assertEqual(p.update(200),[])
        event=p.confirm(200)[0]
        self.assertEqual(event['phase'],'pull');self.assertTrue(event['confirmed'])
        self.assertAlmostEqual(event['reference_force_N'],.0980665)
        self.assertEqual(p.update(202.99),[])
        self.assertEqual(p.update(203)[0]['phase'],'setup_release')
        self.assertEqual(p.update(300),[])
        self.assertEqual(p.confirm(300)[0]['phase'],'release')
        self.assertEqual(p.update(304.99),[])
        self.assertEqual(p.update(305)[0]['phase'],'complete')
        self.assertEqual(p.confirmations,3)

    def test_confirmation_during_timed_phase_cannot_approve_future_stage(self):
        p=PullProtocol(cycles=1,weights_g=[10],countdown_s=0,baseline_s=1)
        p.start(0);p.update(0);p.confirm(1)
        self.assertEqual(p.confirm(1.5),[])
        self.assertEqual(p.update(2)[0]['phase'],'setup_pull')
        self.assertEqual(p.update(20),[])

    def test_auto_advance_can_retain_timed_known_weight_mode(self):
        p=PullProtocol(cycles=1,weights_g=[10],countdown_s=0,baseline_s=1,require_enter=False)
        p.start(0)
        self.assertEqual(p.update(0)[0]['phase'],'baseline')
        self.assertEqual(p.update(1)[0]['phase'],'pull')
        self.assertEqual(p.confirmations,0)

    def test_reject_invalid_protocol_before_recording(self):
        for kwargs in (dict(cycles=0),dict(pull_s=0),dict(release_s=-1),dict(countdown_s=-1),dict(countdown_s=float('nan'))):
            with self.assertRaises(ValueError):PullProtocol(**kwargs)


if __name__=='__main__':unittest.main()
