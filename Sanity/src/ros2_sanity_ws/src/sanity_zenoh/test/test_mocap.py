import json
from pathlib import Path
import socket
import sys
import tempfile
import time
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
from sanity_zenoh.mocap import MocapEstimator, MocapOutlier, state_age
from sanity_zenoh.local_policy import StateInbox, ActorProcess


class MocapTests(unittest.TestCase):
    def test_calibrated_pose_world_twist_quaternion_order_and_sign(self):
        world = Rotation.from_euler("x", 90, degrees=True)
        marker_base = Rotation.from_euler("z", 90, degrees=True)
        est = MocapEstimator(world_quat=world.as_quat(), marker_to_base_xyz=(.1, 0, 0),
                             marker_to_base_quat=marker_base.as_quat(), filter_tau_s=0)
        start = 10**12
        for i in range(10):
            angle = .6 * i * .01
            marker = Rotation.from_euler("z", angle)
            q = marker.as_quat() * (-1 if i % 2 else 1)
            stamp = start + i * 10**7
            packet = est.update("map", "sanity09", stamp, [i*.01, 0, 0], q, stamp+10**6, stamp)
        expected_rotation = world * marker * marker_base
        expected_position = world.apply(np.array([.09, 0, 0]) + marker.apply([.1, 0, 0]))
        np.testing.assert_allclose(packet["state"][:3], expected_position)
        np.testing.assert_allclose(Rotation.from_quat(np.roll(packet["state"][3:7], -1)).as_matrix(), expected_rotation.as_matrix())
        np.testing.assert_allclose(packet["state"][10:13], world.apply([0, 0, .6]), atol=1e-12)
        velocity = world.apply(np.array([1., 0, 0]) + np.cross([0, 0, .6], marker.apply([.1, 0, 0])))
        np.testing.assert_allclose(packet["state"][7:10], velocity, atol=.0002)
        self.assertTrue(packet["ready"])

    def test_independent_filters_use_source_dt_without_smoothing_pose(self):
        legacy=MocapEstimator(filter_tau_s=.02)
        separate=MocapEstimator(filter_tau_s=.02,angular_rate_filter_tau_s=.05)
        stamp=10**12;position=np.array([0.,0.,1.]);angle=0.;velocity=np.zeros(3);omega=np.zeros(3)
        for i in range(2):
            for estimator in (legacy,separate):
                estimator.update('map','sanity09',stamp+i*10_000_000,position,[0.,0.,0.,1.],stamp+i*10_000_000,stamp+i*10_000_000)
        stamp+=10_000_000
        for dt_ns in (7_000_000,13_000_000,5_000_000,17_000_000):
            dt=dt_ns/1e9;stamp+=dt_ns;raw_v=np.array([.2,.1,0.]);raw_w=np.array([0.,0.,1.])
            position+=raw_v*dt;angle+=dt;q=Rotation.from_euler('z',angle).as_quat()
            old=legacy.update('map','sanity09',stamp,position,q,stamp,stamp)
            packet=separate.update('map','sanity09',stamp,position,q,stamp,stamp)
            velocity+=(-np.expm1(-dt/.02))*(raw_v-velocity)
            omega+=(-np.expm1(-dt/.05))*(raw_w-omega)
            np.testing.assert_array_equal(packet['state'][:10],old['state'][:10])
            np.testing.assert_allclose(packet['state'][:3],position)
            np.testing.assert_allclose(packet['state'][7:10],velocity,atol=1e-12)
            np.testing.assert_allclose(packet['state'][10:13],omega,atol=1e-12)
            np.testing.assert_allclose(Rotation.from_quat(np.roll(packet['state'][3:7],-1)).as_matrix(),Rotation.from_quat(q).as_matrix())
            self.assertEqual(packet['stamp_ns'],stamp)
            self.assertEqual(packet['filter_tau_s'],{'linear_velocity':.02,'angular_rate':.05})
        self.assertLess(np.linalg.norm(packet['state'][10:13]),np.linalg.norm(old['state'][10:13]))


    def test_angular_filter_can_be_disabled_and_rejects_invalid_constants(self):
        for tau in (-.01,float('nan'),float('inf')):
            with self.assertRaisesRegex(ValueError,'filter'):
                MocapEstimator(angular_rate_filter_tau_s=tau)
        estimator=MocapEstimator(filter_tau_s=.02,angular_rate_filter_tau_s=0.)
        start=10**12
        for i in range(5):
            stamp=start+i*10_000_000
            packet=estimator.update('map','sanity09',stamp,[0.,0.,1.],[0.,0.,0.,1.],stamp,stamp)
        stamp+=10_000_000
        packet=estimator.update('map','sanity09',stamp,[.01,0.,1.],Rotation.from_euler('x',.01).as_quat(),stamp,stamp)
        np.testing.assert_allclose(packet['state'][10:13],[1.,0.,0.],atol=1e-12)
        self.assertLess(packet['state'][7],1.)
        stamp+=50_000_000
        packet=estimator.update('map','sanity09',stamp,[.02,0.,1.],[0.,0.,0.,1.],stamp,stamp)
        self.assertFalse(packet['ready'])
        np.testing.assert_array_equal(packet['state'][7:13],np.zeros(6))


    def test_frozen_out_of_order_wrong_frame_stale_and_future(self):
        est = MocapEstimator()
        stamp = 10**12
        for i in range(5):
            packet = est.update("map", "sanity09", stamp+i*10**7, [0,0,1], [0,0,0,1], stamp+i*10**7, stamp+i*10**7)
        self.assertTrue(packet["ready"])
        self.assertIsNone(est.update("map", "other", stamp+5*10**7, [0,0,1], [0,0,0,1], stamp+5*10**7, 99))
        self.assertIsNone(est.update("map", "sanity09", stamp+4*10**7, [0,0,1], [0,0,0,1], stamp+4*10**7, 99))
        with self.assertRaises(ValueError):
            state_age(packet, packet["receipt_ns"]+40_000_000)
        for delta in (-2_000_001, 40_000_000):
            with self.assertRaises(ValueError):
                est.update("map", "sanity09", stamp+5*10**7, [0,0,1], [0,0,0,1], stamp+5*10**7+delta, 99)
        recovered = est.update("map", "sanity09", stamp+20*10**7, [0,0,1], [0,0,0,1], stamp+20*10**7, 200)
        self.assertFalse(recovered["ready"])

    def test_small_future_clock_skew_preserves_raw_stamp_and_receipt_expiry(self):
        for skew_ns in (1,150_000,270_000,2_000_000):
            est=MocapEstimator()
            for i in range(5):
                stamp=10**12+i*10_000_000
                packet=est.update('map','sanity09',stamp,[0.,0.,1.],[0.,0.,0.,1.],stamp-skew_ns,stamp)
            self.assertTrue(packet['ready'])
            self.assertEqual(packet['stamp_ns'],stamp)
            self.assertEqual(packet['source_age_s'],0.)
            self.assertAlmostEqual(packet['raw_source_age_s'],-skew_ns/1e9)
            self.assertAlmostEqual(packet['future_clock_skew_s'],skew_ns/1e9)
            self.assertAlmostEqual(state_age(packet,stamp+39_000_000)[1],.039)
            with self.assertRaisesRegex(ValueError,'expired'):
                state_age(packet,stamp+40_000_000)
            self.assertIsNone(est.update('map','sanity09',stamp,[0.,0.,1.],[0.,0.,0.,1.],stamp+1_000_000,stamp+1_000_000))
            with self.assertRaisesRegex(ValueError,'future'):
                est.update('map','sanity09',stamp+10_000_000,[0.,0.,1.],[0.,0.,0.,1.],stamp+10_000_000-2_000_001,stamp+10_000_000)
        for tolerance in (-.001,.002001,float('nan')):
            with self.assertRaises(ValueError):MocapEstimator(max_future_s=tolerance)

    def test_invalid_pose_jump_and_ages_fail(self):
        for q in ([0,0,0,0], [0,0,0,2], [0,0,float("nan"),1]):
            with self.assertRaises(ValueError):
                MocapEstimator(world_quat=q)
        est = MocapEstimator()
        est.update("map","sanity09",10**12,[0,0,1],[0,0,0,1],10**12,0)
        with self.assertRaises(ValueError):
            est.update("map","sanity09",10**12+10**7,[1,0,1],[0,0,0,1],10**12+10**7,10**7)
        packet = dict(version=1, ready=True, source_age_s=.02, receipt_ns=100_000_000,
                      state=[0,0,1,1,0,0,0,0,0,0,0,0,0])
        self.assertAlmostEqual(state_age(packet, 110_000_000)[1], .03)
        for now in (99_000_000,120_000_000):
            with self.assertRaises(ValueError):
                state_age(packet,now)

    def test_isolated_outlier_keeps_valid_state_without_refresh_and_recovers(self):
        est = MocapEstimator()
        start = 10**12
        for i in range(8):
            stamp = start+i*5_000_000
            packet = est.update("map","sanity09",stamp,[i*.001,0.,1.],[0.,0.,0.,1.],stamp,stamp)
        previous, count = est.previous, est.count
        velocity, omega = est.velocity.copy(), est.omega.copy()
        for position, quaternion in (([.008,0.,1.],Rotation.from_euler('x',12,degrees=True).as_quat()),
                                     ([1.,0.,1.],[0.,0.,0.,1.])):
            with self.assertRaises(MocapOutlier):
                est.update("map","sanity09",stamp+5_000_000,position,quaternion,stamp+5_000_000,stamp+5_000_000)
            self.assertIs(est.previous,previous)
            self.assertEqual(est.count,count)
            np.testing.assert_array_equal(est.velocity,velocity)
            np.testing.assert_array_equal(est.omega,omega)
        self.assertEqual(packet['stamp_ns'],stamp)
        self.assertAlmostEqual(state_age(packet,stamp+39_000_000)[1],.039)
        with self.assertRaisesRegex(ValueError,'expired'):
            state_age(packet,stamp+40_000_000)
        recovered = est.update("map","sanity09",stamp+10_000_000,[.009,0.,1.],[0.,0.,0.,1.],stamp+10_000_000,stamp+10_000_000)
        self.assertTrue(recovered['ready'])
        self.assertEqual(recovered['stamp_ns'],stamp+10_000_000)
        np.testing.assert_allclose(recovered['state'][7:10],[.2,0.,0.],atol=1e-12)

    def test_repeated_outliers_do_not_keep_state_alive_past_freshness_limit(self):
        est = MocapEstimator()
        start = 10**12
        for i in range(5):
            stamp = start+i*5_000_000
            packet = est.update("map","sanity09",stamp,[0.,0.,1.],[0.,0.,0.,1.],stamp,stamp)
        for gap in (5,10,15,20,25,30,35,40):
            now = stamp+gap*1_000_000
            with self.assertRaises(MocapOutlier):
                est.update("map","sanity09",now,[1.,0.,1.],[0.,0.,0.,1.],now,now)
        with self.assertRaisesRegex(ValueError,'expired'):
            state_age(packet,stamp+40_000_000)
        recovered = est.update("map","sanity09",stamp+45_000_000,[0.,0.,1.],[0.,0.,0.,1.],stamp+45_000_000,stamp+45_000_000)
        self.assertFalse(recovered['ready'])

    def test_partial_worker_reply_obeys_deadline(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory)/"worker.py"
            source.write_text('import sys,time,json\nprint(json.dumps({"prefix":sys.prefix}),flush=True)\nfor line in sys.stdin:\n sys.stdout.write("{");sys.stdout.flush();time.sleep(10)\n')
            actor = ActorProcess(sys.executable, source, "unused", Path(directory)/"worker.log")
            try:
                self.assertEqual(actor.info["prefix"],sys.prefix)
                begin = time.monotonic()
                with self.assertRaises(TimeoutError):
                    actor.call("infer",deadline=begin+.02)
                self.assertLess(time.monotonic()-begin,.15)
            finally:
                actor.close()

    def test_ipc_has_one_owner_and_does_not_refresh_replayed_state(self):
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory)/"state.sock")
            inbox = StateInbox(path)
            client = socket.socket(socket.AF_UNIX,socket.SOCK_DGRAM)
            try:
                with self.assertRaises(ValueError):
                    StateInbox(path)
                packet = dict(version=1, ready=True, stamp_ns=100, source_age_s=0., receipt_ns=time.monotonic_ns(),
                              tracked_frame="sanity09", frames_confirmed=False,
                              state=[0,0,1,1,0,0,0,0,0,0,0,0,0])
                client.sendto(json.dumps(packet).encode(),path)
                until = time.monotonic()+1
                while inbox.packet is None and time.monotonic()<until:
                    time.sleep(.001)
                self.assertEqual(inbox.snapshot()[2]["stamp_ns"],100)
                packet["receipt_ns"] = time.monotonic_ns()
                client.sendto(json.dumps(packet).encode(),path)
                time.sleep(.005)
                self.assertLess(inbox.snapshot()[2]["receipt_ns"],packet["receipt_ns"])
                client.sendto(b'{"ready":false,"error":"lost tracking"}',path)
                time.sleep(.005)
                with self.assertRaisesRegex(ValueError,"lost tracking"):
                    inbox.snapshot()
            finally:
                client.close()
                inbox.close()


if __name__ == "__main__":
    unittest.main()
