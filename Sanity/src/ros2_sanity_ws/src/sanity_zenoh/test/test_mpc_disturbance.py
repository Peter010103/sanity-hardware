import importlib.util
from pathlib import Path
import unittest
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[6]
PATH=ROOT/'loco_mjx/experiments/sanity_ctbr_sim2sim/mpc_disturbance.py'
spec=importlib.util.spec_from_file_location('mpc_bias_tests',PATH)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def hover_state_and_command(bias,mass=.06947):
    specific=np.array([0.,0.,9.81])-np.asarray(bias)
    z=specific/np.linalg.norm(specific);y=np.cross(z,[1.,0.,0.]);y/=np.linalg.norm(y)
    rotation=Rotation.from_matrix(np.column_stack([np.cross(y,z),y,z]))
    state=np.r_[0.,0.,1.,rotation.as_quat()[[3,0,1,2]],np.zeros(6)]
    command=np.r_[mass*np.linalg.norm(specific),np.zeros(3)]
    return state,command


CONTEXT={'allow_learning':True,'height_above_ground_m':1.,'reference_velocity':[0.]*3,'reference_acceleration':[0.]*3}


def train(observer,bias,duration=12.,context=None):
    state,command=hover_state_and_command(bias,observer.mass)
    context=context or CONTEXT
    start=1_000_000_000
    observer.commit(start-10_000_000,command,True)
    for i in range(int(duration/.02)):
        stamp=start+i*20_000_000
        observer.observe(state,stamp,context)
        observer.commit(stamp+10_000_000,command,True)
    return observer.estimate.copy()


class DisturbanceObserverTests(unittest.TestCase):
    def observer(self,**extra):return module.HorizontalAccelerationBias({'enabled':True,**extra},.06947)

    def test_xy_bias_sign_units_and_vertical_exclusion(self):
        for bias in ([.35,-.22,0.],[-.3,.25,0.]):
            observer=self.observer();estimate=train(observer,bias)
            np.testing.assert_allclose(estimate,bias,atol=.002)
            self.assertEqual(estimate[2],0.)
            self.assertGreater(observer.updates,20)

    def test_per_drone_estimates_do_not_share_state(self):
        first=self.observer();second=self.observer()
        train(first,[.3,0.,0.]);train(second,[0.,-.4,0.])
        np.testing.assert_allclose(first.estimate,[.3,0.,0.],atol=.002)
        np.testing.assert_allclose(second.estimate,[0.,-.4,0.],atol=.002)
        first.reset();np.testing.assert_allclose(first.estimate,0.)
        self.assertLess(second.estimate[1],-.39)

    def test_bias_norm_is_bounded(self):
        observer=self.observer(max_bias_m_s2=.2)
        train(observer,[.5,.4,0.],duration=15.)
        self.assertAlmostEqual(np.linalg.norm(observer.estimate),.2,places=6)

    def test_ground_motion_reference_and_disabled_output_freeze_learning(self):
        for change in [{'allow_learning':False},{'height_above_ground_m':.05},
                       {'reference_velocity':[.3,0.,0.]},{'reference_acceleration':[.5,0.,0.]}]:
            observer=self.observer();context={**CONTEXT,**change}
            train(observer,[.3,0.,0.],context=context)
            np.testing.assert_allclose(observer.estimate,0.)

    def test_saturation_gate_holds_bias_without_windup(self):
        observer=self.observer();train(observer,[.3,0.,0.]);before=observer.estimate.copy()
        state,command=hover_state_and_command([-.3,0.,0.]);start=20_000_000_000
        for i in range(300):
            stamp=start+i*20_000_000
            observer.observe(state,stamp,CONTEXT)
            observer.commit(stamp+10_000_000,command,False)
        np.testing.assert_allclose(observer.estimate,before)

    def test_source_gap_and_repeat_do_not_reset_learned_bias(self):
        observer=self.observer();train(observer,[.3,0.,0.]);before=observer.estimate.copy()
        stamp,state,_=observer.previous
        observer.observe(state,stamp,CONTEXT)
        self.assertEqual(observer.last['reason'],'repeated_source_timestamp')
        observer.observe(state,stamp+100_000_000,CONTEXT)
        self.assertEqual(observer.last['reason'],'source_clock_gap')
        np.testing.assert_allclose(observer.estimate,before)

    def test_uncommitted_shadow_predictions_cannot_enter_estimator(self):
        observer=self.observer();state,command=hover_state_and_command([.3,0.,0.])
        for i in range(50):observer.observe(state,1_000_000_000+i*20_000_000,CONTEXT)
        self.assertEqual(observer.last['reason'],'missing_selected_command')
        np.testing.assert_allclose(observer.estimate,0.)

    def test_command_integration_is_time_aligned_and_excludes_future_input(self):
        observer=self.observer()
        state=np.r_[0.,0.,1.,1.,0.,0.,0.,np.zeros(6)]
        observer.commit(900_000_000,[.6,0.,0.,0.],True)
        observer.commit(1_010_000_000,[.7,0.,0.,0.],True)
        observer.commit(1_100_000_000,[1.,0.,0.,0.],True)
        predicted,reason=observer.predicted_delta_velocity(1_000_000_000,1_020_000_000,state,state)
        self.assertIsNone(reason)
        self.assertAlmostEqual(predicted[2],(.6*.01+.7*.01)/observer.mass-9.81*.02)
        np.testing.assert_allclose(predicted[:2],0.)

    def test_invalid_configuration_and_input_are_rejected(self):
        for config in [{'enabled':1},{'time_constant_s':.01},{'max_bias_m_s2':20.},{'unknown':1.}]:
            with self.assertRaises(ValueError):module.HorizontalAccelerationBias(config,.06947)
        observer=self.observer()
        with self.assertRaises(ValueError):observer.observe([float('nan')]*13,1,CONTEXT)
        state,command=hover_state_and_command([0.,0.,0.])
        with self.assertRaises(ValueError):observer.observe(state,0,CONTEXT)
        with self.assertRaises(ValueError):observer.observe(state,1,{**CONTEXT,'reference_velocity':[float('nan')]*3})
        with self.assertRaises(ValueError):observer.observe(state,1,{**CONTEXT,'reference_acceleration':[0.,0.]})
        observer.commit(1,[.7,0.,0.,0.],True)
        with self.assertRaises(ValueError):observer.commit(1,[.7,0.,0.,0.],True)
