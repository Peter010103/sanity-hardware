import json
from pathlib import Path
import unittest

import numpy as np
from flight_prediction import FlightRatePrediction

ROOT=Path(__file__).resolve().parents[3]
CONFIG=ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_pitch_loop.json'


class FlightPredictionTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(CONFIG.read_text());self.model=FlightRatePrediction(self.config)


    def test_delay_approximation_preserves_measured_band(self):
        for axis,(gain,wn,damping,zero,tau) in enumerate(self.model.coefficients):
            for frequency in [.5,1.,2.,3.,4.]:
                s=2j*np.pi*frequency
                exact=gain*wn**2*(1+zero*s)/(s**2+2*damping*wn*s+wn**2)*np.exp(-3*tau*s)
                actual=self.model.c[axis]@np.linalg.solve(s*np.eye(6)-self.model.a[axis],self.model.b[axis])
                self.assertLess(abs(abs(actual/exact)-1.),.015)
                self.assertLess(abs(np.angle(actual/exact)),.001)


    def test_observer_recovers_rates_without_using_future_commands(self):
        model=self.model;truth=np.zeros((2,6));history=[];errors=[];filtered_errors=[]
        for tick in range(1000):
            t=tick*.005;command=np.r_[.7,.3*np.sin(2*np.pi*2*t),.3*np.sin(2*np.pi*3*t),0.]
            history.append((t,.7,command))
            if tick:
                ad,bd=model.transition(50);truth=np.einsum('aij,aj->ai',ad,truth)+bd*history[-2][2][1:3,None]
            future=(t+1.,.7,np.array([.7,8.,8.,0.]))
            result=model.observe(t,truth[:,5],history+[future],0.)
            actual=np.sum(model.c*truth,axis=1)
            if tick>100:
                errors.append(np.array(result['rate_rad_s'])-actual);filtered_errors.append(truth[:,5]-actual)
        self.assertLess(np.sqrt(np.mean(np.array(errors)**2)),.005)
        self.assertLess(np.sqrt(np.mean(np.array(errors)**2)),np.sqrt(np.mean(np.array(filtered_errors)**2))/10)


    def test_output_delay_is_not_applied_twice(self):
        model=self.model;history=[(0.,.7,np.array([.7,0.,0.,0.])),(.02,.7,np.array([.7,.3,.3,0.]))]
        model.observe(0.,[0.,0.],history,.015)
        result=model.observe(.03,[0.,0.],history,.015)
        np.testing.assert_allclose(result['state'],0.,atol=1e-12)
        result=model.observe(.05,[0.,0.],history,.015)
        self.assertGreater(np.linalg.norm(result['state']),0.)


    def test_measurement_filter_and_profile_are_explicit(self):
        self.assertEqual(self.model.filter_tau,.05)
        self.assertEqual(len(self.model.info['sha256']),64)
        self.assertIsNone(self.model.info['command_output_filter'])
        with self.assertRaisesRegex(ValueError,'timestamp'):
            FlightRatePrediction({**self.config,'rate_prediction_timestamp':'unknown'})
        with self.assertRaisesRegex(ValueError,'filter'):
            FlightRatePrediction({**self.config,'angular_rate_filter_tau_s':0.})




class MPCFlightDynamicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        from controller import PitchLoopMPC
        cls.configuration=json.loads(CONFIG.read_text());cls.controller=PitchLoopMPC(cls.configuration)


    def test_nonlinear_prediction_matches_independent_small_rate_response(self):
        controller=self.controller;model=FlightRatePrediction(self.configuration)
        hover=self.configuration['mass_kg']*9.81
        state=controller.augment(np.r_[0.,0.,.5,1.,0.,0.,0.,np.zeros(6),hover]);linear=np.zeros((2,6));errors=[]
        for tick in range(200):
            rates=np.array([.3 if 10<=tick<80 else 0.,-.2 if 100<=tick<160 else 0.])
            state=controller.forecast(state,np.r_[hover,rates,0.],.005)
            ad,bd=model.transition(50);linear=np.einsum('aij,aj->ai',ad,linear)+bd*rates[:,None]
            expected=np.sum(model.c*linear,axis=1);errors.append(state[10:12]-expected)
        self.assertLess(np.sqrt(np.mean(np.array(errors)**2)),.002)


    def test_cache_reuses_voltage_and_mission_settings_with_runtime_bounds(self):
        from controller import PitchLoopMPC
        config={**self.configuration,'scope':'simulation_only_pitch_loop','hover_only':True,'host_output_delay_s':.02,
            'physical_force_range_N':[.05,2.91],'maximum_thrust_N':2.8,'pitch_limit_rad_s':9.4}
        other=PitchLoopMPC(config)
        self.assertTrue(other.info['cache_hit']);self.assertEqual(other.info['cache'],self.controller.info['cache'])
        self.assertEqual(other.info['runtime_force_bounds_N'],[.05,2.91])
        np.testing.assert_allclose(other.bounds[:,1],[2.8,2.3,9.4,2.6])
        hover=config['mass_kg']*9.81;state=np.r_[0.,0.,.5,1.,0.,0.,0.,np.zeros(6),hover]
        preview=np.tile(np.r_[state,hover,0.,0.,0.],(other.nodes+1,1))
        for _ in range(4):command=other.command(state,preview)
        self.assertTrue(other.last['valid']);np.testing.assert_allclose(command,[hover,0.,0.,0.],atol=1e-6)
        key=json.loads((Path(other.info['cache'])/'build.json').read_text())
        self.assertEqual(key['flight_rate_profile_sha256'],other.flight_rate.info['sha256'])
        self.assertNotIn('physical_force_range_N',key['configuration'])
        self.assertNotIn('hover_only',key['configuration'])


    def test_observer_state_layout_and_profile_match(self):
        controller=self.controller;model=FlightRatePrediction(self.configuration)
        observer=model.observe(0.,[.1,-.2],[(0.,.7,np.array([.7,0.,0.,0.]))],0.)
        state=controller.augment(np.r_[0.,0.,.5,1.,0.,0.,0.,np.zeros(6),.7],observer)
        self.assertEqual(state.shape,(22,));np.testing.assert_allclose(state[10:12],[.1,-.2])
        with self.assertRaisesRegex(ValueError,'profile mismatch'):
            controller.augment(state[:14],{**observer,'profile_sha256':'wrong'})


if __name__=='__main__':unittest.main()
