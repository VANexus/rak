"""Pure-vision road following: road mask -> traced centerline -> /cmd_vel.

Scope (user decision 2026-11-18): ONLY the front camera + road segmentation +
"drive where the road is". No map, no lidar, no localization, no Nav2.
Junction policy: keep the current heading (straight through).

Run discipline (ADR-0011 single /cmd_vel writer): this node is the writer ONLY
while Nav2 / rak_wbc / chassis_tui are NOT running. `enabled` defaults to false
so a stray launch can never move the chassis on its own.

Math lives in road_follow_math.py (numpy-only, testable without ROS).
"""

import time

import numpy as np

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from .road_follow_math import (
    FOLLOW_DEFAULTS,
    TRACE_DEFAULTS,
    follow_command,
    trace_centerline,
)

NODE_DEFAULTS = {
    "enabled": False,
    "mask_topic": "/road_perception/road_mask",
    "cmd_topic": "/cmd_vel",
    "control_hz": 20.0,
    "max_mask_age_s": 0.5,
}

PARAM_DEFAULTS = {**NODE_DEFAULTS, **TRACE_DEFAULTS, **FOLLOW_DEFAULTS}

PARAM_TYPES = {
    "enabled": bool,
    "mask_topic": str,
    "cmd_topic": str,
    "control_hz": float,
    "max_mask_age_s": float,
    "row_step_frac": float,
    "branch_gate": float,
    "gap_px": int,
    "wide_run_frac": float,
    "bottom_search_frac": float,
    "max_vx": float,
    "max_vy": float,
    "max_wz": float,
    "k_lateral": float,
    "k_vy": float,
    "k_heading": float,
    "lookahead_frac": float,
    "curv_slow": float,
    "min_vx_scale": float,
    "slow_lateral": float,
    "spin_wz": float,
    "spin_search": bool,
}


class RoadFollowNode(Node):

    def __init__(self):
        super().__init__("road_follow_node")
        self._params = {}
        for name, default in PARAM_DEFAULTS.items():
            ptype = PARAM_TYPES[name]
            if ptype is bool:
                self.declare_parameter(name, bool(default))
            elif ptype is int:
                self.declare_parameter(name, int(default))
            elif ptype is float:
                self.declare_parameter(name, float(default))
            else:
                self.declare_parameter(name, default)
            self._params[name] = self.get_parameter(name).value

        self._enabled = bool(self._params["enabled"])
        if not self._enabled:
            self.get_logger().warning(
                "enabled:=false — publishing nothing. Pass enabled:=true ONLY "
                "when Nav2/rak_wbc/chassis_tui are down (single /cmd_vel writer)."
            )
        self._last_obs = ([], None, False)
        self._last_mask_time = None
        self._warned_at = {}
        self._cmd_pub = self.create_publisher(Twist, self._params["cmd_topic"], 10)
        self._mask_sub = self.create_subscription(
            Image,
            self._params["mask_topic"],
            self._mask_callback,
            qos_profile_sensor_data,
        )
        period = 1.0 / max(1.0, float(self._params["control_hz"]))
        self.create_timer(period, self._tick)
        self._last_cmd = (0.0, 0.0, 0.0)

    def _mask_callback(self, message: Image) -> None:
        h, w = int(message.height), int(message.width)
        if h == 0 or w == 0 or message.encoding != "mono8":
            self._warn("encoding", f"unsupported mask encoding {message.encoding!r}")
            return
        mask = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(h, w)
        self._last_obs = trace_centerline(mask, self._params)
        self._last_mask_time = time.monotonic()

    def _tick(self) -> None:
        vx, wz = 0.0, 0.0
        fresh = (
            self._last_mask_time is not None
            and time.monotonic() - self._last_mask_time
            <= float(self._params["max_mask_age_s"])
        )
        if self._enabled and fresh:
            vx, vy, wz = follow_command(*self._last_obs, self._params)
        elif self._enabled and not fresh:
            self._warn("stale", "road mask stale — zero velocity")
        if (vx, vy, wz) != self._last_cmd:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self._cmd_pub.publish(twist)
            self._last_cmd = (vx, vy, wz)

    def _warn(self, key, message, interval_s=5.0):
        now = time.monotonic()
        if now - self._warned_at.get(key, 0.0) >= interval_s:
            self.get_logger().warning(message)
            self._warned_at[key] = now


def main(args=None) -> None:
    import rclpy

    rclpy.init(args=args)
    node = RoadFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
