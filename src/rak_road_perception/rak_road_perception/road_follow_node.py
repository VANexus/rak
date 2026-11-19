"""Pure-vision road following: road mask -> traced centerline -> /cmd_vel.

Scope (user decision 2026-11-18): ONLY the front camera + road segmentation +
"drive where the road is". No map, no lidar, no localization, no Nav2.

2026-11-19 (field symptoms: cannot take a corner / no left-right at a square
corner / never settles in the lane centre): the LEFT/RIGHT judgement moved INTO
`trace_centerline` (it appends the corner arm to the polyline), because a
polyline that turns is the only thing no stock package can produce for you.
The MOTION is stock: run `road_goal_node` with nav2 (`nav2_rotation_shim_controller`
for the in-place rotation + `nav2_mppi_controller` for tracking). This node is
the no-Nav2 fallback and keeps a plain memoryless `follow_command()`.
See docs/how-to/road_follow_split.md.

ROS-native split (user decision 2026-11-19): this node runs on the LOGIC host
(the workstation) and consumes `/road_perception/road_mask` produced by
`roadseg_node` on the Jetson over DDS; the Jetson only drives. Launch both halves
with `rak_bringup/launch/road_follow.launch.py`.

Run discipline (ADR-0011 single /cmd_vel writer): this node is the writer ONLY
while Nav2 / rak_wbc / chassis_tui are NOT running. `enabled` defaults to false
so a stray launch can never move the chassis on its own.

Math lives in road_follow_math.py (numpy-only, testable without ROS).
"""

import time

import numpy as np

from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image

from .road_follow_math import (
    CORNER_DEFAULTS,
    FOLLOW_DEFAULTS,
    TRACE_DEFAULTS,
    follow_command,
    render_trace_debug,
    trace_centerline,
)
from .params import declare_all

NODE_DEFAULTS = {
    "enabled": False,
    "mask_topic": "/road_perception/road_mask",
    "cmd_topic": "/cmd_vel",
    "control_hz": 20.0,
    "max_mask_age_s": 0.3,
    "debug_image": True,
    "debug_topic": "/road_perception/centerline_debug",
    "debug_hz": 10.0,
    # TwistStamped output for mecanum_drive_controller (~/reference takes
    # TwistStamped, verified against jazzy's header on the board 2026-11-19).
    # Two motion gates: enabled AND publish_reference. Both default false.
    "publish_reference": False,
    "reference_topic": "/mecanum_drive_controller/reference",
    "reference_frame": "base_link",
}

PARAM_DEFAULTS = {
    **NODE_DEFAULTS, **TRACE_DEFAULTS, **FOLLOW_DEFAULTS, **CORNER_DEFAULTS,
}




class RoadFollowNode(Node):

    def __init__(self):
        super().__init__("road_follow_node")
        self._params = declare_all(self, PARAM_DEFAULTS)

        self._enabled = bool(self._params["enabled"])
        if not self._enabled:
            self.get_logger().warning(
                "enabled:=false — publishing nothing. Pass enabled:=true ONLY "
                "when Nav2/rak_wbc/chassis_tui are down (single /cmd_vel writer)."
            )
        self._last_mask = None
        self._last_mask_time = None
        self._last_obs = ([], None, False)
        self._last_trace = []
        self._last_debug_time = 0.0
        self._warned_at = {}

        self._cmd_pub = self.create_publisher(Twist, self._params["cmd_topic"], 10)
        self._ref_pub = None
        if bool(self._params["publish_reference"]):
            self._ref_pub = self.create_publisher(
                TwistStamped, self._params["reference_topic"], 10
            )
            self.get_logger().warning(
                "publish_reference:=true — writing "
                f"{self._params['reference_topic']} (mecanum_drive_controller). "
                "This is a motion path: allow_motion:=true on chassis_bringup too."
            )
        self._mask_sub = self.create_subscription(
            Image,
            self._params["mask_topic"],
            self._mask_callback,
            qos_profile_sensor_data,
        )
        # RViz's Image display subscribes RELIABLE; depth 1 keeps latency low.
        self._debug_pub = None
        if bool(self._params["debug_image"]):
            self._debug_pub = self.create_publisher(
                Image,
                self._params["debug_topic"],
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
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
        # Trace once per mask, not once per control tick: the control timer runs
        # at 20 Hz while masks arrive at ~18-30 Hz, so re-tracing in _tick would
        # only redo identical work.
        self._last_obs = trace_centerline(mask, self._params)
        self._last_trace = self._last_obs[0]
        self._last_mask = mask
        self._last_mask_time = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()

        vx, vy, wz = 0.0, 0.0, 0.0
        fresh = (
            self._last_mask is not None
            and self._last_mask_time is not None
            and now - self._last_mask_time <= float(self._params["max_mask_age_s"])
        )
        if self._enabled and fresh:
            vx, vy, wz = follow_command(*self._last_obs, self._params)
        elif self._enabled and not fresh:
            self._warn("stale", "road mask stale — zero velocity")
            self._last_obs = ([], None, False)
            self._last_trace = []

        # Publish EVERY tick while enabled -- not only on change: downstream
        # consumers time out (cmd_vel_sim 0.15 s, the MCU 100 ms deadman), so a
        # constant cruise command must be re-sent at the control rate.
        # Change-only publishing looked alive on paper and delivered exactly one
        # Twist on the wire (measured 2026-11-19: 2 publishers, 0 messages in a
        # 12 s watch).
        #
        # But when `enabled:=false` NOTHING is published, not even zeros: ADR-0011
        # is single-writer by NODE, and a 20 Hz stream of zero Twist makes this
        # node a /cmd_vel writer that can silently out-vote whoever is supposed to
        # own the topic (Nav2 / rak_wbc / chassis_tui).
        if self._enabled:
            twist = Twist()
            twist.linear.x = vx
            twist.linear.y = vy
            twist.angular.z = wz
            self._cmd_pub.publish(twist)
            self._last_cmd = (vx, vy, wz)
            if self._ref_pub is not None:
                stamped = TwistStamped()
                stamped.header.stamp = self.get_clock().now().to_msg()
                stamped.header.frame_id = str(self._params["reference_frame"])
                stamped.twist = twist
                self._ref_pub.publish(stamped)

        self._maybe_publish_debug(now)

    def _maybe_publish_debug(self, now: float) -> None:
        if self._debug_pub is None or self._last_mask is None:
            return
        hz = max(1.0, float(self._params["debug_hz"]))
        if now - self._last_debug_time < 1.0 / hz:
            return
        self._last_debug_time = now
        img = render_trace_debug(self._last_mask, self._last_trace)
        message = Image()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "camera_optical_frame"
        message.height, message.width = int(img.shape[0]), int(img.shape[1])
        message.encoding = "mono8"
        message.is_bigendian = 0
        message.step = int(img.shape[1])
        message.data = img.tobytes()
        self._debug_pub.publish(message)

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
