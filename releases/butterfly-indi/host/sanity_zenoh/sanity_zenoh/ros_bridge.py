"""ROS 2 physical command gateway; ROS middleware does not need to change."""

from dataclasses import asdict
import json
import math

import rclpy
from rclpy.node import Node
from sanity_msgs.msg import CtbrCommand, DirectThrustCommand
from std_msgs.msg import String
from udp_broadcaster.ctbr_protocol import CtbrMapping
from sanity_zenoh.actuator import DirectThrustMapping
from sanity_zenoh.client import SanitySender
from sanity_zenoh.protocol import CTBR, DIRECT


class ZenohSenderNode(Node):
    def __init__(self):
        super().__init__("sanity_zenoh_sender")
        defaults = {
            "drone_id": 9,
            "mode": "ctbr",
            "listen": "tcp/0.0.0.0:7447",
            "connect": "",
            "telemetry_listen": "",
            "transport_diagnostics": False,
            "rate_hz": 100.0,
            "command_timeout_s": 0.1,
            "rate_full_scale_deg_s": [600.0, 600.0, 600.0],
            "rate_signs": [1.0, 1.0, -1.0],
            "model_rate_limits_rad_s": [2.3, 2.3, 2.6],
            "max_thrust_newtons": 3.2,
            "thrust_newtons": [0.0],
            "throttle_rc": [1000.0],
            "actuator_model_path": "",
            "model_to_output": [0, 0, 0, 0],
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        values = {name: self.get_parameter(name).value for name in defaults}
        if self.get_parameter("use_sim_time").value:
            raise ValueError(
                "Hardware Zenoh bridge requires the live ROS clock (use_sim_time=false)"
            )
        if values["mode"] not in ("ctbr", "direct_thrust"):
            raise ValueError("mode must be ctbr or direct_thrust")
        self.mode = CTBR if values["mode"] == "ctbr" else DIRECT
        self.device = values["drone_id"]
        self.timeout = values["command_timeout_s"]
        if not math.isfinite(self.timeout) or not 0 < self.timeout <= 0.1:
            raise ValueError("command_timeout_s must be in (0, 0.1]")
        self.last_stamp = -1
        mapping = CtbrMapping(
            **{
                name: tuple(values[name])
                for name in (
                    "rate_full_scale_deg_s",
                    "rate_signs",
                    "model_rate_limits_rad_s",
                    "thrust_newtons",
                    "throttle_rc",
                )
            },
            max_thrust_newtons=values["max_thrust_newtons"],
        )
        direct = (
            DirectThrustMapping(
                values["actuator_model_path"], values["model_to_output"]
            )
            if values["actuator_model_path"]
            else None
        )
        self.sender = SanitySender(
            self.device,
            self.mode,
            values["listen"],
            values["connect"] or None,
            mapping,
            direct,
            values["rate_hz"],
            telemetry_listen=values["telemetry_listen"] or None,
            diagnostics=values["transport_diagnostics"],
        )
        topic = f"sanity{self.device:02d}/"
        message = CtbrCommand if self.mode == CTBR else DirectThrustCommand
        self.subscription = self.create_subscription(
            message,
            topic + ("ctbr_command" if self.mode == CTBR else "direct_thrust_command"),
            self.command_update,
            1,
        )
        self.status_publisher = self.create_publisher(String, topic + "zenoh_status", 1)
        self.status_timer = self.create_timer(0.02, self.publish_status)
        self.get_logger().info(
            f"Zenoh drone {self.device}: {values['mode']}; waiting for receiver and explicit disable/enable sequence"
        )

    def command_update(self, msg):
        try:
            if msg.header.frame_id != f"sanity{self.device:02d}/base_link":
                raise ValueError("Wrong model frame; expected sanityNN/base_link (FLU)")
            stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= age < self.timeout or stamp <= self.last_stamp:
                raise ValueError("Command timestamp is stale, future-dated or repeated")
            remaining = self.timeout - age
            if self.mode == CTBR:
                self.sender.ctbr(
                    msg.armed,
                    msg.collective_thrust_newtons,
                    (
                        msg.body_rates_rad_s.x,
                        msg.body_rates_rad_s.y,
                        msg.body_rates_rad_s.z,
                    ),
                    remaining,
                )
            else:
                self.sender.direct_thrust(
                    msg.enabled, tuple(msg.rotor_thrust_newtons), remaining
                )
            self.last_stamp = stamp
        except (ValueError, TypeError) as error:
            self.sender.invalidate(str(error))
            self.get_logger().warning(str(error), throttle_duration_sec=1.0)

    def publish_status(self):
        status = self.sender.receiver_status()
        data = asdict(status) if status else {"connected": False}
        data["sender_error"] = self.sender.last_error
        self.status_publisher.publish(String(data=json.dumps(data)))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ZenohSenderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.sender.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
