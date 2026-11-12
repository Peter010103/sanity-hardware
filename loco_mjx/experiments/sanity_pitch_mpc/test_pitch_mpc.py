import json
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from reference import PitchLoopReference


class PitchReferenceTests(unittest.TestCase):
    def test_geometry_and_full_inversions_in_both_directions(self):
        for direction in (-1,1):
            reference=PitchLoopReference(laps=5,direction=direction)
            samples=reference.sample(np.linspace(0.,reference.duration,4001))
            x,z=samples[:,0],samples[:,2]
            np.testing.assert_allclose((x/reference.a)**2+((z-reference.bottom-reference.b)/reference.b)**2,1.,atol=3e-5)
            self.assertGreaterEqual(z.min(),.5-1e-6);self.assertLessEqual(z.max(),1.80001)
            np.testing.assert_allclose(np.linalg.norm(samples[:,3:7],axis=1),1.,atol=1e-12)
            self.assertAlmostEqual(reference.info['force_reference_turns'],5.,places=5)
            rotations=Rotation.from_quat(samples[:,[4,5,6,3]])
            inverted=rotations.as_matrix()[:,2,2]<-.7
            self.assertEqual(np.count_nonzero(inverted&~np.r_[False,inverted[:-1]]),5)
            self.assertLess(reference.info['peak_feedforward_pitch_command_rad_s'],9.5)
            self.assertLess(reference.info['peak_force_command_N'],2.85)


    def test_hover_transitions_and_acceleration_are_consistent(self):
        reference=PitchLoopReference()
        for t in (0.,reference.duration):
            sample=reference.sample(t)
            np.testing.assert_allclose(sample[:3],[0.,0.,.5],atol=1e-7)
            np.testing.assert_allclose(sample[7:13],0.,atol=1e-5)
            self.assertAlmostEqual(sample[13],.06947*9.81,places=5)
        for t in np.linspace(reference.hold+.01,reference.hold+reference.moving_s-.01,40):
            before,now,after=reference.sample(np.array([t-.002,t,t+.002]))
            np.testing.assert_allclose((after[:3]-before[:3])/.004,now[7:10],atol=.0015)
            acceleration=(after[7:10]-before[7:10])/.004
            body_z=Rotation.from_quat(now[[4,5,6,3]]).as_matrix()[:,2]
            np.testing.assert_allclose(body_z*now[13]/.06947-[0,0,9.81],acceleration,atol=.02)


    def test_selected_reference_has_additional_ceiling_margin(self):
        reference=PitchLoopReference(.4769230769230769,.62,5.856231172724236,laps=5)
        self.assertAlmostEqual(reference.info['top_m'],1.74)
        self.assertAlmostEqual(reference.info['force_reference_turns'],5.)
        self.assertLess(reference.info['peak_feedforward_pitch_command_rad_s'],9.5)
        self.assertLess(reference.info['peak_force_command_N'],2.85)


    def test_invalid_or_non_inverting_reference_is_rejected(self):
        for config in ({'vertical_radius':.8},{'phase_rate':1.},{'direction':0},{'laps':0},{'ramp_s':10.}):
            with self.assertRaises(ValueError):PitchLoopReference(**config)


class ControllerScopeTests(unittest.TestCase):
    def test_controller_requires_simulation_scope_before_build(self):
        from controller import PitchLoopMPC
        for config in ({},{'scope':'flight'}):
            with self.assertRaises(ValueError):PitchLoopMPC(config)





class RunInReferenceTests(unittest.TestCase):
    def setUp(self):
        from run_in_reference import RunInPitchReference
        self.cfg=json.loads((Path(__file__).resolve().parents[3]/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_pitch_loop.json').read_text())
        self.cls=RunInPitchReference

    def test_both_directions_force_and_kinematic_consistency(self):
        for direction in (-1,1):
            r=self.cls(self.cfg,laps=3,direction=direction)
            self.assertAlmostEqual(r.info['force_reference_turns'],3.)
            self.assertLess(r.info['initial_position_m'][0]*direction,-.8)
            self.assertGreater(r.info['final_position_m'][0]*direction,.8)
            for t in np.linspace(r.hold+.01,r.hold+r.moving_s-.01,150):
                before,now,after=r.sample([t-.001,t,t+.001])
                np.testing.assert_allclose((after[:3]-before[:3])/.002,now[7:10],atol=.002)
                force=Rotation.from_quat(now[[4,5,6,3]]).as_matrix()[:,2]*now[13]/self.cfg['mass_kg']
                np.testing.assert_allclose((after[7:10]-before[7:10])/.002,force-[0,0,9.81],atol=.025)
            for t in (r.hold,r.loop_start,r.loop_end,r.hold+r.moving_s):
                left,right=r.sample([t-.0001,t+.0001])
                self.assertLess(np.max(abs(left-right)),.07)
            self.assertGreater(min(r.info['nominal_constraint_margins'].values()),0.)

    def test_level_run_in_develops_forward_speed_before_pitch_entry(self):
        r=self.cls(self.cfg,laps=2)
        times=np.linspace(r.hold,r.hold+r.acceleration_s,401)
        samples=r.sample(times)
        np.testing.assert_allclose(samples[:,2],.5,atol=1e-12)
        np.testing.assert_allclose(samples[:,9],0.,atol=1e-8)
        self.assertGreaterEqual(samples[:,7].min(),-1e-7)
        self.assertAlmostEqual(samples[-1,7],2.4,places=4)
        for t in (r.hold+r.acceleration_s,r.loop_end+r.pull_up_s):
            left,right=r.sample([t-.000001,t+.000001])
            self.assertLess(np.max(abs(left-right)),.002)
        np.testing.assert_allclose(r.sample(r.duration)[7:13],0.,atol=1e-8)


    def test_model_limits_and_height_are_checked_before_flight(self):
        for cfg,bottom in (({**self.cfg,'maximum_thrust_N':2.},.5),({**self.cfg,'pitch_limit_rad_s':8.},.5),(self.cfg,.9),
                ({**self.cfg,'torque_reserve_fraction':.001},.5)):
            with self.assertRaisesRegex(ValueError,'infeasible'):self.cls(cfg,bottom=bottom)


if __name__=='__main__':unittest.main()
