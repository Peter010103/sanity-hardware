from sanity_zenoh.standalone_paths import load_json
import json
from pathlib import Path
import unittest
import numpy as np
from sanity_zenoh.pitch_mission import PitchMission

ROOT=Path(__file__).resolve().parents[6]
CONFIG=Path(__file__).resolve().parents[1]/'config/sanity09_pitch_loop.json'


class PitchMissionTests(unittest.TestCase):
    def setUp(self):
        self.config={**load_json(CONFIG),"reference_mode":"ellipse","host_selection_delay_s":0.,"host_output_delay_s":0.}
        self.m=PitchMission(ROOT,self.config)
        self.state=np.r_[0.,0.,.03,1.,0.,0.,0.,np.zeros(6)]
        self.m.begin(100.,self.state)

    def result(self,now,valid=True):
        return {'request_ns':int(now*1e9),'reference_epoch':self.m.epoch,'ctbr':[self.config['mass_kg']*9.81,0.,0.,0.],'mpc':{'valid':valid}}

    def handover(self):
        self.state[2]=.5
        for now in [112.,113.02]:
            command,_=self.m.choose(now,self.state,self.result(now),3.1);self.m.commit(now,command)
        self.assertIsNotNone(self.m.epoch)

    def test_hover_gate_then_full_attitude_reference(self):
        self.handover()
        request=self.m.inference_request(113.04,self.state,.01)
        self.assertEqual(len(request['state']),14);self.assertEqual(np.shape(request['preview']),(21,18))
        self.assertEqual(request['reference_epoch'],self.m.epoch)
        q=self.m.reference.table[:,3:7]
        self.assertLess((1-2*(q[:,1]**2+q[:,2]**2)).min(),-.99)

    def test_stale_or_invalid_MPC_cannot_trigger_inverted_PD_fallback(self):
        self.handover()
        for r in [self.result(113.04,False),self.result(113.)]:
            with self.assertRaisesRegex(ValueError,'stale/invalid'):self.m.choose(113.06,self.state,r,3.1)

    def test_manual_landing_during_loop_waits_for_upright_recovery(self):
        self.handover();self.m.request_landing('operator_interrupt')
        self.m.motion_epoch=self.m.epoch+self.m.reference.hold
        now=self.m.epoch+3.
        self.state[3:7]=[0.,0.,1.,0.];self.state[2]=1.5
        self.m.choose(now,self.state,self.result(now),3.1)
        self.assertIsNone(self.m.landing);self.assertEqual(self.m.phase,'pitch_loops')

    def test_normal_recovery_landing_and_two_second_low_height_dwell(self):
        self.handover()
        self.m.motion_epoch=self.m.epoch+self.m.reference.hold
        recovery=self.m.epoch+self.m.reference.hold+self.m.reference.moving_s+2.
        for now in [recovery,recovery+1.02]:self.m.choose(now,self.state,self.result(now),3.1)
        self.assertIsNotNone(self.m.landing)
        self.state[2]=.06
        start=recovery+1.04
        for now in np.arange(start,start+2.7,.02):self.m.choose(float(now),self.state,self.result(float(now)),3.1)
        self.assertTrue(self.m.done)

    def test_model_frame_rates_and_causal_selected_history(self):
        self.m.commit(100.02,np.array([.6,.1,.2,.3]))
        req=self.m.inference_request(100.04,self.state,.01)
        self.assertTrue(all(d>0 for d,_ in req['prediction_segments']))
        np.testing.assert_allclose(req['sample_command'],[.6,.1,.2,.3])
        self.assertGreaterEqual(req['state'][-1],0.)
        self.assertLessEqual(req['state'][-1],.6)
        self.state[3:7]=[np.sqrt(.5),0.,0.,np.sqrt(.5)]
        self.state[10:13]=[1.,2.,3.]
        rotated=self.m.inference_request(100.04,self.state,.01)
        np.testing.assert_allclose(rotated['state'][10:13],[2.,-1.,3.],atol=1e-12)

    def test_takeoff_timeout_enters_landing_once_and_advances(self):
        self.state[2]=.5
        self.m.choose(135.02,self.state,None,3.1)
        self.assertEqual(self.m.landing_requested,'takeoff_gate_timeout')
        self.m.choose(135.04,self.state,None,3.1)
        landing=self.m.landing;start=landing.start
        self.assertIsNone(self.m.epoch)
        phases=[]
        for now in np.arange(135.06,144.06,.02):
            command,ref=self.m.choose(float(now),self.state,None,3.1)
            self.assertIs(self.m.landing,landing)
            self.assertEqual(self.m.landing.start,start)
            phases.append(self.m.phase)
        self.assertIn('landing_return',phases)
        self.assertIn('landing_descend',phases)
        self.assertLess(ref['position'][2],.5)

    def test_operator_landing_without_mpc_handover_is_not_recreated(self):
        self.state[2]=.4
        self.m.request_landing('operator_interrupt')
        self.m.choose(105.,self.state,None,3.1)
        landing=self.m.landing
        for now in np.arange(105.02,112.,.02):self.m.choose(float(now),self.state,None,3.1)
        self.assertIs(self.m.landing,landing)
        self.assertEqual(self.m.landing.start,105.)

    def test_hover_command_disagreement_remains_a_visible_gate(self):
        self.state[2]=.5
        for now in [112.,113.02,114.04]:
            r=self.result(now);r['ctbr'][2]=-1.2
            self.m.choose(now,self.state,r,3.1)
        self.assertIsNone(self.m.epoch)
        self.assertEqual(self.m.gate['blocking'],['rate_match'])

    def test_relaxed_command_match_still_requires_one_second(self):
        self.state[2]=.5
        for now in [112.,112.6]:
            r=self.result(now);r['ctbr'][0]+=.25;r['ctbr'][2]=.8
            self.m.choose(now,self.state,r,3.1)
            self.assertIsNone(self.m.epoch)
            self.assertEqual(self.m.gate['blocking'],[])
        now=113.02
        r=self.result(now);r['ctbr'][0]+=.25;r['ctbr'][2]=.8
        self.m.choose(now,self.state,r,3.1)
        self.assertEqual(self.m.epoch,now)
        self.assertEqual(self.m.gate['limits'],{'rate_tolerance_rad_s':1.,'thrust_tolerance_N':.3,'dwell_s':1.})

    def test_stale_result_resets_relaxed_handover_dwell(self):
        self.state[2]=.5
        self.m.choose(112.,self.state,self.result(112.),3.1)
        self.m.choose(112.8,self.state,self.result(112.7),3.1)
        self.assertIsNone(self.m.settled_since)
        self.assertIn('mpc_valid_fresh',self.m.gate['blocking'])
        self.m.choose(113.02,self.state,self.result(113.02),3.1)
        self.assertIsNone(self.m.epoch)
        self.m.choose(114.04,self.state,self.result(114.04),3.1)
        self.assertIsNotNone(self.m.epoch)

    def test_excess_thrust_mismatch_blocks_relaxed_handover(self):
        self.state[2]=.5
        for now in [112.,113.02]:
            r=self.result(now);r['ctbr'][0]+=.35
            self.m.choose(now,self.state,r,3.1)
        self.assertIsNone(self.m.epoch)
        self.assertEqual(self.m.gate['blocking'],['thrust_match'])

    def test_handover_tolerances_reject_invalid_configuration(self):
        for key in ['handover_rate_tolerance_rad_s','handover_thrust_tolerance_N']:
            for value in [0.,-1.,float('nan'),float('inf')]:
                with self.subTest(key=key,value=value),self.assertRaisesRegex(ValueError,'Handover tolerances'):
                    PitchMission(ROOT,{**self.config,key:value})

    def test_pitch_only_calibration_removes_confirmed_ground_tilt(self):
        import yaml
        from scipy.spatial.transform import Rotation
        from sanity_zenoh.mocap import MocapEstimator
        params=yaml.safe_load((CONFIG.parent/'pitch_loop_sequence.yaml').read_text())['sanity_sequence']['ros__parameters']
        estimator=MocapEstimator(marker_to_base_quat=params['marker_to_base_xyzw'])
        ground=(Rotation.from_euler('z',-.01)*Rotation.from_euler('xyz',[-.6579962537991834,2.271053618151669,0.],degrees=True)).as_quat()
        packet=estimator.update('map','sanity09',1000000000,[0.,0.,.03],ground,1000000000,1000000000)
        corrected=Rotation.from_quat(np.array(packet['state'])[[4,5,6,3]]).as_euler('xyz')
        np.testing.assert_allclose(corrected,[0.,0.,-.01],atol=1e-10)
        ordinary=yaml.safe_load((CONFIG.parent/'circle_sequence.yaml').read_text())['sanity_sequence']['ros__parameters']
        self.assertEqual(ordinary['marker_to_base_xyzw'],[0.,0.,0.,1.])

    def test_calibrated_mocap_preserves_full_pitch_inversions(self):
        import yaml
        from scipy.spatial.transform import Rotation
        from sanity_zenoh.mocap import MocapEstimator
        params=yaml.safe_load((CONFIG.parent/'pitch_loop_sequence.yaml').read_text())['sanity_sequence']['ros__parameters']
        estimator=MocapEstimator(marker_to_base_quat=params['marker_to_base_xyzw'])
        marker_from_body=Rotation.from_quat(params['marker_to_base_xyzw'])
        for t in np.arange(0.,self.m.reference.duration,.01):
            ref=self.m.reference.sample(float(t))
            body=Rotation.from_quat(ref[[4,5,6,3]])
            raw=body*marker_from_body.inv()
            stamp=int(1e9+t*1e9)
            packet=estimator.update('map','sanity09',stamp,ref[:3],raw.as_quat(),stamp,stamp)
            actual=Rotation.from_quat(np.array(packet['state'])[[4,5,6,3]])
            self.assertLess((actual.inv()*body).magnitude(),1e-12)
            np.testing.assert_allclose(packet['state'][:3],ref[:3],atol=1e-12)

    def test_predicted_ceiling_rejection_is_distinct_from_stale_result(self):
        self.handover()
        now=113.04
        r=self.result(now-.02,False)
        r['mpc'].update(status=0,predicted_height_max_m=2.002875,predicted_height_min_m=.74,total_ms=1.5,input_violation=0.)
        with self.assertRaisesRegex(ValueError,'predicted ceiling=2.0029m'):
            self.m.choose(now,self.state,r,3.1)
        self.assertAlmostEqual(self.m.mpc_rejection['result_age_s'],.02)
        self.assertEqual(self.m.mpc_rejection['result']['mpc']['status'],0)
        self.assertFalse(any('result age=' in v for v in self.m.mpc_rejection['reasons']))
        self.assertIsNone(self.m.landing)

    def test_stale_result_logs_age_and_keeps_last_request(self):
        self.handover()
        r=self.result(113.02)
        with self.assertRaisesRegex(ValueError,'result age=60.0ms'):
            self.m.choose(113.08,self.state,r,3.1)
        self.assertIs(self.m.mpc_rejection['result'],r)

    def test_host_delay_advances_preview_and_uses_only_queued_commands(self):
        cfg={**self.config,'host_selection_delay_s':.02,'host_output_delay_s':.015}
        m=PitchMission(ROOT,cfg);m.begin(100.,self.state);m.epoch=100.
        m.commit(100.02,np.array([.6,.1,.2,.3]));m.commit(100.04,np.array([.7,.2,.3,.4]))
        self.assertAlmostEqual(m.prediction_delay,.035)
        self.assertAlmostEqual(m.force,.6*(1-np.exp(-.005/cfg['collective_tau_up_s'])))
        request=m.inference_request(100.04,self.state,.01)
        self.assertAlmostEqual(sum(d for d,_ in request['prediction_segments']),.045)
        np.testing.assert_allclose(request['sample_command'],np.zeros(4))
        self.assertAlmostEqual(request['state'][-1],0.)
        m.motion_epoch=100.
        request=m.inference_request(100.04,self.state,.01)
        expected=m.reference.sample(m.reference.hold+.075)
        np.testing.assert_allclose(request['preview'][0],expected,atol=1e-10)
        commands=[tuple(u) for _,u in request['prediction_segments']]
        self.assertIn((0.,0.,0.,0.),commands)
        self.assertIn((.6,.1,.2,.3),commands)
        self.assertIn((.7,.2,.3,.4),commands)

    def test_causal_force_estimate_accounts_for_nonzero_output_queue(self):
        cfg={**self.config,'host_output_delay_s':.015}
        m=PitchMission(ROOT,cfg);m.begin(100.,self.state)
        m.commit(100.02,np.array([.6,0.,0.,0.]))
        self.assertEqual(m.force_at(100.03),0.)
        self.assertAlmostEqual(m.force_at(100.04),.6*(1-np.exp(-.005/cfg['collective_tau_up_s'])))
        for t in [100.025,100.03,100.034]:np.testing.assert_allclose(m.applied_command_at(t),0.)
        np.testing.assert_allclose(m.applied_command_at(100.036),[.6,0.,0.,0.])


class RunInMissionTests(unittest.TestCase):
    def setUp(self):
        self.cfg=load_json(CONFIG);self.m=PitchMission(ROOT,self.cfg,laps=2)
        self.ground=np.r_[0.,0.,.03,1.,0.,0.,0.,np.zeros(6)];self.m.begin(100.,self.ground)
        self.hover=self.ground.copy();self.hover[:3]=self.m.reference.sample(0.)[:3]

    def result(self, now):
        return {'request_ns':int((now-.02)*1e9),'reference_epoch':self.m.epoch,'ctbr':[self.cfg['mass_kg']*9.81,0.,0.,0.],
            'mpc':{'valid':True,'rollout_vs_raw_state_max_error':0.}}

    def enter_mpc(self):
        for now in (122.,123.02):self.m.choose(now,self.hover,self.result(now),3.1)
        self.assertIsNotNone(self.m.epoch)

    def test_pd_moves_behind_before_mpc_handover(self):
        self.m.choose(112.,self.ground,None,3.1)
        self.assertIsNone(self.m.epoch)
        self.enter_mpc()
        self.assertLess(self.hover[0],-.8)
        self.assertIsNone(self.m.motion_epoch)

    def test_preview_frozen_until_measured_mpc_hover_is_stable(self):
        self.enter_mpc()
        for now in np.arange(123.04,126.04,.02):
            bad=self.hover.copy();bad[7]=.3
            self.m.choose(float(now),bad,self.result(float(now)),3.1)
        self.assertIsNone(self.m.motion_epoch)
        preview=np.array(self.m.inference_request(126.04,self.hover,.01)['preview'])
        np.testing.assert_allclose(preview,np.tile(preview[0],(21,1)))
        for now in np.arange(126.06,127.12,.02):self.m.choose(float(now),self.hover,self.result(float(now)),3.1)
        self.assertIsNotNone(self.m.motion_epoch)
        self.assertEqual(self.m.phase,'run_in')
        self.assertGreater(np.ptp(np.array(self.m.inference_request(127.14,self.hover,.01)['preview'])[:,0]),.1)

    def test_oscillatory_hover_times_out_to_single_pd_landing_without_run_in(self):
        self.enter_mpc();self.hover[7]=.3
        now=self.m.epoch+12.02
        self.m.choose(now,self.hover,self.result(now),3.1)
        self.assertIsNone(self.m.motion_epoch)
        self.assertEqual(self.m.landing.reason,'mpc_hover_gate_timeout')
        landing=self.m.landing
        self.m.choose(now+.02,self.hover,None,3.1)
        self.assertIs(self.m.landing,landing)

    def test_hover_only_qualification_does_not_start_maneuver(self):
        self.m.config['hover_only']=True;self.enter_mpc()
        for now in np.arange(125.04,126.12,.02):self.m.choose(float(now),self.hover,self.result(float(now)),3.1)
        self.assertIsNone(self.m.motion_epoch)
        self.assertEqual(self.m.landing.reason,'hover_qualification_complete')


if __name__=='__main__':unittest.main()
