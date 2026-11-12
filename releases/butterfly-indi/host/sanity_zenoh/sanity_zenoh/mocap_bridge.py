"""Receive ROS 2 TF through DDS and relay only the selected state over local IPC."""

import json
import os
from pathlib import Path
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sanity_zenoh.mocap import MocapEstimator, MocapOutlier, state_age


def default_socket():
    return str(Path(os.environ.get("XDG_RUNTIME_DIR", f"/tmp/sanity-{os.getuid()}")) / "sanity01/state.sock")


class MocapBridge(Node):
    def __init__(self):
        super().__init__("sanity_mocap")
        defaults = {"tf_topic": "/tf", "parent_frame": "map", "tracked_frame": "sanity09",
                    "state_socket": default_socket(), "world_from_map_xyzw": [0., 0., 0., 1.],
                    "marker_to_base_xyz": [0., 0., 0.], "marker_to_base_xyzw": [0., 0., 0., 1.],
                    "velocity_filter_tau_s": .02, "angular_rate_filter_tau_s": .05, "frames_confirmed": False}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        v = {name: self.get_parameter(name).value for name in defaults}
        if self.get_parameter("use_sim_time").value:
            raise ValueError("Hardware mocap requires use_sim_time=false")
        self.estimator = MocapEstimator(v["parent_frame"], v["tracked_frame"], v["world_from_map_xyzw"],
                                        v["marker_to_base_xyz"], v["marker_to_base_xyzw"],
                                        v["velocity_filter_tau_s"], frames_confirmed=v["frames_confirmed"],
                                        angular_rate_filter_tau_s=v["angular_rate_filter_tau_s"])
        self.destination = v["state_socket"]
        self.ipc = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.ipc.setblocking(False)
        self.last_packet = {"ready": False, "error": "Waiting for selected TF"}
        self.seen_frames = set()
        self.last_tf_rx_ns = None
        self.rejected_samples = 0
        self.last_rejection = None
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE)
        self.subscription = self.create_subscription(TFMessage, v["tf_topic"], self.receive_tf, qos)
        self.status = self.create_publisher(String, "sanity01/mocap_status", 1)
        self.timer = self.create_timer(.5, self.publish_status)
        self.get_logger().info(f"DDS TF {v['parent_frame']} -> {v['tracked_frame']}; local IPC {self.destination}")

    def receive_tf(self, message):
        self.last_tf_rx_ns = time.monotonic_ns()
        self.seen_frames.update((tf.header.frame_id, tf.child_frame_id) for tf in message.transforms)
        for tf in message.transforms:
            if (tf.header.frame_id, tf.child_frame_id) != (self.estimator.parent, self.estimator.child):
                continue
            t, q = tf.transform.translation, tf.transform.rotation
            try:
                packet = self.estimator.update(tf.header.frame_id, tf.child_frame_id,
                    tf.header.stamp.sec * 10**9 + tf.header.stamp.nanosec,
                    [t.x, t.y, t.z], [q.x, q.y, q.z, q.w], self.get_clock().now().nanoseconds,
                    time.monotonic_ns())
                if packet is None:
                    continue
            except MocapOutlier as error:
                self.rejected_samples += 1
                self.last_rejection = {"reason":str(error), **error.diagnostics}
                continue
            except (ValueError, TypeError) as error:
                packet = {"version": 1, "ready": False, "error": str(error)}
            self.last_packet = packet
            try:
                self.ipc.sendto(json.dumps(packet, allow_nan=False).encode(), self.destination)
            except (FileNotFoundError, ConnectionRefusedError, BlockingIOError):
                pass

    def publish_status(self):
        packet = {**self.last_packet, "rejected_samples":self.rejected_samples, "last_rejection":self.last_rejection}
        if packet.get("ready"):
            try:
                state_age(packet, time.monotonic_ns())
            except ValueError as error:
                packet.update(ready=False, error=str(error))
        packet["observed_frames"] = sorted(" -> ".join(frame) for frame in self.seen_frames)
        packet["last_tf_arrival_age_s"] = (time.monotonic_ns() - self.last_tf_rx_ns) / 1e9 if self.last_tf_rx_ns else None
        if "receipt_ns" in packet:
            packet["local_age_s"] = (time.monotonic_ns() - packet["receipt_ns"]) / 1e9
        self.status.publish(String(data=json.dumps(packet)))

    def destroy_node(self):
        self.ipc.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MocapBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
