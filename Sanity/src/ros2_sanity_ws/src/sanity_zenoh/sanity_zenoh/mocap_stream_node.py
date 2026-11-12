"""ROS TF -> calibrated compact Zenoh state. Observation only; no commands."""
import json
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from std_msgs.msg import String
from .mocap import MocapEstimator
from .mocap_stream import MocapStreamPublisher
from .transport import SanityTransport


class MocapStreamNode(Node):
    def __init__(self, **kwargs):
        super().__init__('sanity_mocap_stream', **kwargs)
        defaults=dict(device=9,tf_topic='/tf',parent_frame='map',tracked_frame='sanity09',
            world_from_map_xyzw=[0.,0.,0.,1.],marker_to_base_xyz=[0.,0.,0.],marker_to_base_xyzw=[0.,0.,0.,1.],
            frames_confirmed=False,velocity_filter_tau_s=.02,maximum_rate_hz=100.,clock_rate_hz=5.,clock_endpoint='',
            listen=['udp/0.0.0.0:7447','tcp/127.0.0.1:7449'],connect='')
        for name,value in defaults.items():self.declare_parameter(name,value)
        v={name:self.get_parameter(name).value for name in defaults}
        if self.get_parameter('use_sim_time').value:raise ValueError('Real mocap stream requires use_sim_time=false')
        if not v['frames_confirmed']:raise ValueError('Confirm world Z-up / body FLU and marker calibration with frames_confirmed:=true')
        self.estimator=MocapEstimator(v['parent_frame'],v['tracked_frame'],v['world_from_map_xyzw'],
            v['marker_to_base_xyz'],v['marker_to_base_xyzw'],v['velocity_filter_tau_s'],frames_confirmed=True)
        self.transport=SanityTransport(v['listen'],v['connect'] or None)
        self.stream=None
        try:
            self.stream=MocapStreamPublisher(self.transport.session,v['device'],v['maximum_rate_hz'],clock_rate_hz=v['clock_rate_hz'],clock_endpoint=v['clock_endpoint'] or None)
            self.rejections=0;self.last_rejection=None
            self.status_pub=self.create_publisher(String,f"sanity{v['device']:02d}/mocap_stream_status",1)
            self.subscription=self.create_subscription(TFMessage,v['tf_topic'],self.receive,
                QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE))
            self.timer=self.create_timer(.5,self.publish_status)
            self.get_logger().info(f"Mocap only: {v['parent_frame']} -> {v['tracked_frame']} -> sanity/{v['device']:02d}/state/mocap; no FC/control writes")
        except BaseException:
            try:
                if self.stream is not None:self.stream.close()
            finally:
                self.transport.close()
                super().destroy_node()
            raise

    def receive(self,message):
        for tf in message.transforms:
            now=self.get_clock().now().nanoseconds;t,q=tf.transform.translation,tf.transform.rotation
            try:
                packet=self.estimator.update(tf.header.frame_id,tf.child_frame_id,
                    tf.header.stamp.sec*10**9+tf.header.stamp.nanosec,[t.x,t.y,t.z],[q.x,q.y,q.z,q.w],now,
                    time.monotonic_ns())
                if packet is not None:self.stream.send_estimate(packet,self.get_clock().now().nanoseconds)
            except (ValueError,TypeError) as error:self.rejections+=1;self.last_rejection=str(error)

    def publish_status(self):
        snapshot=self.stream.snapshot();snapshot.update(tf_rejections=self.rejections,last_tf_rejection=self.last_rejection)
        self.status_pub.publish(String(data=json.dumps(snapshot,allow_nan=False)))

    def destroy_node(self):
        self.stream.close();self.transport.close();return super().destroy_node()

def main(args=None):
    rclpy.init(args=args);node=None
    try:node=MocapStreamNode();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':main()
