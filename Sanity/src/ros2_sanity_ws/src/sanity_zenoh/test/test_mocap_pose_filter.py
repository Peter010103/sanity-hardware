import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.mocap import MocapEstimator, state_age


class MocapPoseFilterTests(unittest.TestCase):
    def test_pose_noise_reduced_without_cascading_derivative_filters(self):
        smooth=MocapEstimator(position_filter_tau_s=.01,attitude_filter_tau_s=.01,angular_rate_filter_tau_s=.05)
        original=MocapEstimator(angular_rate_filter_tau_s=.05)
        position_errors=[];angle_errors=[]
        for tick in range(200):
            stamp=10**12+tick*5_000_000;noise=(-1.)**tick
            position=[.002*noise,0.,1.];rotation=Rotation.from_euler('x',.0015*noise)
            packet=smooth.update('map','sanity09',stamp,position,rotation.as_quat(),stamp,stamp)
            raw=original.update('map','sanity09',stamp,position,rotation.as_quat(),stamp,stamp)
            np.testing.assert_allclose(packet['state'][7:13],raw['state'][7:13],atol=1e-12)
            if tick>20:
                position_errors.append(packet['state'][0]);angle_errors.append(Rotation.from_quat(np.roll(packet['state'][3:7],-1)).magnitude())
        self.assertLess(np.std(position_errors),.002*.3)
        self.assertLess(np.sqrt(np.mean(np.array(angle_errors)**2)),.0015*.3)
        self.assertEqual(packet['stamp_ns'],stamp)
        self.assertEqual(packet['pose_filter_tau_s'],{'position':.01,'attitude':.01})


    def test_recorded_delay_compensates_constant_motion_and_full_rotation(self):
        estimator=MocapEstimator(position_filter_tau_s=.01,attitude_filter_tau_s=.01)
        for tick in range(300):
            t=tick*.005;stamp=10**12+tick*5_000_000
            rotation=Rotation.from_euler('y',8.5*t);q=rotation.as_quat()*((-1.)**tick)
            packet=estimator.update('map','sanity09',stamp,[.5*t,0.,1.],q,stamp,stamp)
            if tick>60:
                compensated=np.array(packet['state'][:3])+packet['pose_filter_delay_s']['position']*np.array([.5,0.,0.])
                np.testing.assert_allclose(compensated,[.5*t,0.,1.],atol=1e-10)
                corrected=Rotation.from_quat(np.roll(packet['state'][3:7],-1))*Rotation.from_rotvec([0.,8.5*packet['pose_filter_delay_s']['attitude'],0.])
                self.assertLess((corrected.inv()*rotation).magnitude(),1e-10)
        self.assertAlmostEqual(packet['pose_filter_delay_s']['position'],.005/np.expm1(.5))
        with self.assertRaisesRegex(ValueError,'expired'):state_age(packet,stamp+40_000_000)


    def test_gap_resets_pose_and_delay_without_claiming_ready(self):
        estimator=MocapEstimator(position_filter_tau_s=.01,attitude_filter_tau_s=.01)
        for tick in range(10):
            stamp=10**12+tick*5_000_000
            estimator.update('map','sanity09',stamp,[0.,0.,1.],[0.,0.,0.,1.],stamp,stamp)
        stamp+=50_000_000;rotation=Rotation.from_euler('y',.2)
        packet=estimator.update('map','sanity09',stamp,[.1,0.,1.],rotation.as_quat(),stamp,stamp)
        self.assertFalse(packet['ready']);self.assertEqual(packet['pose_filter_delay_s'],{'position':0.,'attitude':0.})
        np.testing.assert_allclose(packet['state'][:3],[.1,0.,1.])
        self.assertLess((Rotation.from_quat(np.roll(packet['state'][3:7],-1)).inv()*rotation).magnitude(),1e-12)


    def test_invalid_filter_constants_rejected(self):
        for key in ['position_filter_tau_s','attitude_filter_tau_s']:
            for value in [-.01,.041,float('nan'),float('inf')]:
                with self.subTest(key=key,value=value),self.assertRaisesRegex(ValueError,'Pose filter'):
                    MocapEstimator(**{key:value})


if __name__=='__main__':unittest.main()
