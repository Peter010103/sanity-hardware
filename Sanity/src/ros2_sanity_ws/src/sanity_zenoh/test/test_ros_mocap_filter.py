import pytest
pytest.importorskip('geometry_msgs.msg')
import threading
from types import SimpleNamespace
import unittest
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage
from sanity_zenoh.mocap import MocapEstimator, state_age
from sanity_zenoh.ros_sequence import SequenceNode


class RosMocapFilterTests(unittest.TestCase):
    def test_rejected_tf_does_not_replace_live_packet_or_restart_warmup(self):
        now = [10**12]
        events = []
        receiver = SimpleNamespace(estimator=MocapEstimator(), packet=None,
            lock=threading.Lock(), events=events.append,
            get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=now[0])))
        def receive(stamp, quaternion):
            now[0] = stamp
            tf = TransformStamped()
            tf.header.frame_id = 'map';tf.child_frame_id = 'sanity09'
            tf.header.stamp.sec,tf.header.stamp.nanosec = divmod(stamp,10**9)
            tf.transform.translation.z = 1.
            q = tf.transform.rotation
            q.x,q.y,q.z,q.w = map(float,quaternion)
            SequenceNode.receive_tf(receiver,TFMessage(transforms=[tf]))
        for i in range(6):
            receive(10**12+i*5_000_000,[0.,0.,0.,1.])
        previous = receiver.packet
        self.assertTrue(previous['ready'])
        receive(now[0]+5_000_000,Rotation.from_euler('x',12,degrees=True).as_quat())
        self.assertIs(receiver.packet,previous)
        self.assertEqual(events[-1]['kind'],'mocap_rejected_sample')
        self.assertGreater(events[-1]['angular_rate_rad_s'],30.)
        with self.assertRaisesRegex(ValueError,'expired'):
            state_age(receiver.packet,previous['receipt_ns']+40_000_000)
        receive(now[0]+5_000_000,[0.,0.,0.,1.])
        self.assertTrue(receiver.packet['ready'])
        self.assertEqual(receiver.packet['stamp_ns'],now[0])
