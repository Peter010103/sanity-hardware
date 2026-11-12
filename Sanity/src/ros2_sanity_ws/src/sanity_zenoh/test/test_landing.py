from sanity_zenoh.standalone_paths import load_json
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from sanity_zenoh.collective import MotorCollective
from sanity_zenoh.landing import Landing
from sanity_zenoh.recording import EventRecorder

ROOT=Path(__file__).resolve().parents[6]
PROFILE=ROOT/'Sanity/src/ros2_sanity_ws/src/sanity_zenoh/config/sanity09_motor_collective.json'


class CollectiveTests(unittest.TestCase):
    def test_motor_curve_to_armed_rc_and_back_across_voltages(self):
        m=MotorCollective(ROOT/'Sanity/sanity-hardware',load_json(PROFILE))
        for voltage in (7.52,7.8,8.11):
            m.set_voltage(voltage)
            for thrust in (.2,.6483429,1.):
                rc=m.convert(1,True,thrust,(.3,-.5,.1))
                self.assertAlmostEqual(m.thrust_from_rc(rc.throttle),thrust,delta=.006)
                # All four motors share the bench command, so sum each rotor's force.
                u=m.last['external_motor_counts'];q=(u-1350)/100
                if u>=1250:
                    c=m.coefficients
                    expected=m.k*np.sum(((voltage/8)*(c[:,0]+c[:,1]*q+c[:,2]*q*q))**2)
                    self.assertAlmostEqual(thrust,expected,delta=.00002)
        self.assertEqual(m.convert(1,False,0.,(0.,0.,0.)).throttle,1000)

    def test_limits_idle_and_voltage_are_explicit(self):
        m=MotorCollective(ROOT/'Sanity/sanity-hardware',load_json(PROFILE))
        for voltage in (7.,8.3,float('nan')):
            with self.assertRaises(ValueError):m.set_voltage(voltage)
        m.set_voltage(7.8)
        self.assertAlmostEqual(m.limit(3.2),1.221926832297144)
        with self.assertRaises(ValueError):m.convert(1,True,1.3,(0.,0.,0.))
        m.convert(1,True,.0001,(0.,0.,0.))
        self.assertTrue(m.last['below_measured_idle'])
        # Armed minimum throttle still has motor idle; it is not physical zero thrust.
        self.assertGreater(m.thrust_from_rc(1000),0.)

    def test_explicit_voltage_bypass_uses_nominal_only_for_missing_voltage(self):
        profile=load_json(PROFILE)
        m=MotorCollective(ROOT/'Sanity/sanity-hardware',profile,bypass_voltage_check=True)
        m.set_voltage(0.)
        self.assertEqual(m.voltage,8.)
        self.assertEqual(m.reported_voltage,0.)
        rc=m.convert(1,True,.6483429,(0.,0.,0.))
        self.assertGreater(rc.throttle,1000)
        self.assertEqual(m.last['voltage_source'],'assumed_8V_receiver_voltage_unavailable')
        m.set_voltage(8.3)
        self.assertEqual(m.voltage,8.3)
        self.assertAlmostEqual(m.thrust_from_rc(m.convert(1,True,.65,(0.,0.,0.)).throttle),.65,delta=.006)
        with self.assertRaises(ValueError):m.set_voltage(float('nan'))
        with self.assertRaises(ValueError):m.convert(1,True,5.,(0.,0.,0.))

    def test_sag_compensation_changes_rc_not_thrust_target(self):
        profile=load_json(PROFILE)
        a=MotorCollective(ROOT/'Sanity/sanity-hardware',profile)
        b=MotorCollective(ROOT/'Sanity/sanity-hardware',{**profile,'vbat_sag_compensation':0})
        a.set_voltage(7.8);b.set_voltage(7.8)
        self.assertGreater(a.convert(1,True,.65,(0.,0.,0.)).throttle,b.convert(1,True,.65,(0.,0.,0.)).throttle)


class LandingTests(unittest.TestCase):
    def test_reference_is_continuous_and_returns_home_before_descending(self):
        l=Landing(60,[.5,0,1],[0,0],.016,'mission_complete')
        for boundary in (62,l.descend_start,l.descend_start+l.descend_s):
            before=l.sample(boundary-1e-6);after=l.sample(boundary+1e-6)
            np.testing.assert_allclose(before['position'],after['position'],atol=1e-6)
            np.testing.assert_allclose(before['velocity'],after['velocity'],atol=1e-6)
        np.testing.assert_allclose(l.sample(l.descend_start)['position'],[0,0,1])
        self.assertEqual(l.weight(62),0.)
        self.assertGreaterEqual(l.sample(l.timeout)['position'][2],-.005)

    def test_exit_requires_two_continuous_seconds_within_ten_cm(self):
        l=Landing(0,[.5,0,1],[0,0],.016,'test')
        state=np.array([0.,0.,.066,1,0,0,0,.2,0,0,0,0,0])
        for i in range(100):
            self.assertFalse(l.update_touchdown(i*.02,state,15))
            self.assertIsNone(l.touch_time)
        self.assertFalse(l.update_touchdown(2.,state,15))
        self.assertAlmostEqual(l.touch_time,2.)
        self.assertFalse(l.update_touchdown(2.3,state,15))
        self.assertTrue(l.update_touchdown(2.61,state,15))
        self.assertEqual(l.spool(2.61),0.)


    def test_height_excursion_or_gap_restarts_exit_dwell(self):
        state=np.array([0.,0.,.066,1,0,0,0,0,0,0,0,0,0])
        for gap in (False,True):
            l=Landing(0,[0,0,1],[0,0],.016,'test')
            for i in range(90):l.update_touchdown(i*.02,state,0)
            if gap:
                t=2.0
            else:
                high=state.copy();high[2]=.117
                l.update_touchdown(1.8,high,0);t=1.82
            l.update_touchdown(t,state,0)
            for i in range(1,100):
                l.update_touchdown(t+i*.02,state,0)
                self.assertIsNone(l.touch_time)
            l.update_touchdown(t+2.,state,0)
            self.assertAlmostEqual(l.touch_time,t+2.)


    def test_exit_never_counts_before_landing_or_accepts_nonfinite_state(self):
        l=Landing(10,[0,0,1],[0,0],.016,'test')
        state=np.array([0.,0.,.066,1,0,0,0,0,0,0,0,0,0])
        for t in np.arange(0,9.99,.02):l.update_touchdown(t,state,0)
        self.assertIsNone(l.touch_time)
        state[2]=np.nan
        with self.assertRaises(ValueError):l.update_touchdown(10.,state,0)


    def test_async_event_recorder_flushes_all_events(self):
        with tempfile.TemporaryDirectory() as path:
            file=Path(path)/'events.jsonl';r=EventRecorder(file)
            for i in range(500):r({'kind':'test','sequence':i})
            r.close()
            self.assertIsNone(r.error)
            self.assertEqual([json.loads(x)['sequence'] for x in file.read_text().splitlines()],list(range(500)))


class QuickLandingTests(unittest.TestCase):
    def test_quick_reference_shortens_approach_with_bounded_vertical_motion(self):
        standard=Landing(0.,[0.,0.,.45],[0.,0.],.016,'test',velocity=[0.,0.,0.])
        quick=Landing(0.,[0.,0.,.45],[0.,0.],.016,'test',velocity=[0.,0.,0.],speed_profile='quick')
        self.assertLess(quick.descend_start+quick.descend_s,standard.descend_start+standard.descend_s)
        refs=[quick.sample(t) for t in np.arange(0.,8.,.01)]
        self.assertLessEqual(max(abs(r['velocity'][2]) for r in refs),.5)
        self.assertLessEqual(max(abs(r['acceleration'][2]) for r in refs),2.)
        self.assertAlmostEqual(refs[-1]['position'][2],quick.ground-.04)
