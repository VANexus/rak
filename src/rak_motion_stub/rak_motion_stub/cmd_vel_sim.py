"""Non-hardware body-motion and telemetry adapter for software simulation."""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rak_interfaces.msg import BaseTelemetry


class CmdVelSimulation(Node):
    def __init__(self):
        super().__init__("cmd_vel_sim")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("telemetry_topic", "/base/telemetry")
        self.declare_parameter("command_timeout_sec", 0.15)
        self.declare_parameter("max_accel_xy_m_s2", 0.8)
        self.declare_parameter("max_accel_wz_rad_s2", 1.5)
        self._timeout = float(self.get_parameter("command_timeout_sec").value)
        self._accel_xy = float(self.get_parameter("max_accel_xy_m_s2").value)
        self._accel_wz = float(self.get_parameter("max_accel_wz_rad_s2").value)
        self._target = (0.0, 0.0, 0.0)
        self._measured = (0.0, 0.0, 0.0)
        self._command_time = 0.0
        self._last_update = time.monotonic()
        self._sub = self.create_subscription(
            Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self._on_twist,
            10,
        )
        self._pub = self.create_publisher(
            BaseTelemetry, str(self.get_parameter("telemetry_topic").value), 10
        )
        self._timer = self.create_timer(0.02, self._publish)
        self.get_logger().warning(
            "SIMULATION ONLY: telemetry mirrors the final safe Twist topic; not hardware feedback"
        )

    def _on_twist(self, msg):
        values = (msg.linear.x, msg.linear.y, msg.angular.z)
        if not all(math.isfinite(value) for value in values):
            self._target = (0.0, 0.0, 0.0)
        else:
            self._target = values
        self._command_time = time.monotonic()

    @staticmethod
    def _approach(current, target, max_delta):
        return max(current - max_delta, min(current + max_delta, target))

    def _publish(self):
        now = time.monotonic()
        dt = max(0.0, min(now - self._last_update, 0.1))
        self._last_update = now
        target = self._target if now - self._command_time <= self._timeout else (0.0, 0.0, 0.0)
        self._measured = (
            self._approach(self._measured[0], target[0], self._accel_xy * dt),
            self._approach(self._measured[1], target[1], self._accel_xy * dt),
            self._approach(self._measured[2], target[2], self._accel_wz * dt),
        )
        msg = BaseTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.connected = True
        msg.handshake_ok = True
        msg.valid = True
        msg.armed = True
        msg.mode = 1
        msg.measured_twist.linear.x = self._measured[0]
        msg.measured_twist.linear.y = self._measured[1]
        msg.measured_twist.angular.z = self._measured[2]
        msg.age_sec = 0.0
        msg.status = "SIMULATION ONLY: measured_twist is a smoothed command estimate"
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSimulation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
