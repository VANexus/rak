"""Vision centerline -> nav_msgs/Path -> Nav2 FollowPath (MPPI, stock nav2).

MPPI mode of pure-vision road following (user decision 2026-11-18, desktop-
verified with a simulated base): the traced mask centerline is projected to the
flat ground plane (calibrated intrinsics + [AI] mount height), extended to
`path_len_m`, transformed to `odom` via TF, published on /vision_path and sent
to the nav2 FollowPath action, which the stock MPPI controller (Omni model,
nav2_params.yaml FollowPath) tracks and turns into /cmd_vel.

Motion writers: MPPI writes /cmd_vel here — road_follow_node (cmd_vel mode)
MUST be off (ADR-0011). `enabled` defaults to false.
"""

import math
import time

import numpy as np

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

from .road_follow_math import (
    CORNER_DEFAULTS,
    GOAL_DEFAULTS,
    TRACE_DEFAULTS,
    extend_to_length,
    ground_points,
    trace_centerline,
)
from .params import declare_all

NODE_DEFAULTS = {
    "enabled": False,
    "mask_topic": "/road_perception/road_mask",
    "path_topic": "/vision_path",
    "follow_action": "/follow_path",
    "odom_frame": "odom",
    "base_frame": "base_link",
    "resend_period_s": 2.0,
    "path_hz": 5.0,
}

# CORNER_DEFAULTS is here because this node calls the SAME trace_centerline()
# the direct follower does -- without it the corner params would silently fall
# back to the in-code defaults and could not be tuned for the MPPI path mode.
PARAM_DEFAULTS = {**NODE_DEFAULTS, **TRACE_DEFAULTS, **CORNER_DEFAULTS, **GOAL_DEFAULTS}




class RoadGoalNode(Node):

    def __init__(self):
        super().__init__("road_goal_node")
        self._params = declare_all(self, PARAM_DEFAULTS)

        self._enabled = bool(self._params["enabled"])
        if not self._enabled:
            self.get_logger().warning(
                "enabled:=false — no path, no FollowPath goal. Pass enabled:=true "
                "ONLY with road_follow_node OFF (MPPI is the /cmd_vel writer)."
            )
        self._warned_at = {}
        self._last_path = None          # [(d, y_left)] in base frame
        self._sent_path = None          # last BASE-frame path sent to FollowPath
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._path_pub = self.create_publisher(Path, self._params["path_topic"], 10)
        self._mask_sub = self.create_subscription(
            Image, self._params["mask_topic"], self._mask_callback,
            qos_profile_sensor_data,
        )
        self._client = ActionClient(self, FollowPath, self._params["follow_action"])
        period = 1.0 / max(1.0, float(self._params["path_hz"]))
        self.create_timer(period, self._tick)
        self.create_timer(float(self._params["resend_period_s"]), self._resend)

    def _mask_callback(self, message: Image) -> None:
        h, w = int(message.height), int(message.width)
        if h == 0 or w == 0 or message.encoding != "mono8":
            self._warn("encoding", f"unsupported mask encoding {message.encoding!r}")
            return
        params = dict(self._params)
        params["image_width"], params["image_height"] = w, h
        mask = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(h, w)
        pts, _spin, has = trace_centerline(mask, self._params)
        if has and pts:
            self._last_path = extend_to_length(ground_points(pts, params), params)
        else:
            self._last_path = None

    def _to_odom(self, points):
        """Base-frame points -> (stamp, [(x, y)]) in odom; None without TF."""
        try:
            tf = self._tf_buffer.lookup_transform(
                self._params["odom_frame"], self._params["base_frame"],
                rclpy.time.Time(), Duration(seconds=0.2))
        except Exception as exc:  # tf2 raises Lookup/Extrapolation exceptions
            self._warn("tf", f"no {self._params['base_frame']}→{self._params['odom_frame']} TF: {exc}")
            return None
        yaw = math.atan2(
            2.0 * (tf.transform.rotation.w * tf.transform.rotation.z
                   + tf.transform.rotation.x * tf.transform.rotation.y),
            1.0 - 2.0 * (tf.transform.rotation.y ** 2 + tf.transform.rotation.z ** 2))
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        tx, ty = tf.transform.translation.x, tf.transform.translation.y
        return tf.header.stamp, [
            (tx + cos_y * d - sin_y * y_left, ty + sin_y * d + cos_y * y_left)
            for d, y_left in points
        ]

    def _tick(self) -> None:
        if not (self._enabled and self._last_path):
            return
        stamped = self._to_odom(self._last_path)
        if stamped is None:
            return
        stamp, points = stamped
        path = Path()
        path.header.frame_id = self._params["odom_frame"]
        path.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _resend(self) -> None:
        if not self._enabled or not self._last_path:
            return
        if not self._client.wait_for_server(timeout_sec=0.5):
            self._warn("server", "FollowPath action server not up yet")
            return
        stamped = self._to_odom(self._last_path)
        if stamped is None:
            return
        stamp, points = stamped
        goal = FollowPath.Goal()
        goal.controller_id = "FollowPath"
        goal.path.header.frame_id = self._params["odom_frame"]
        goal.path.header.stamp = stamp
        for x, y in points:
            pose = PoseStamped()
            pose.header = goal.path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            goal.path.poses.append(pose)
        # Re-send ONLY when the ROAD changed. Compare in the base frame, never in
        # odom: the odom-frame path shifts on every metre driven even when the
        # road ahead is identical, and each resend preempts the running goal --
        # the preempt window emits zero velocity (measured 2026-11-18:
        # /cmd_vel_smoothed alternated 0.205 / 0.000 at exactly the resend period).
        current = self._last_path
        if self._sent_path is not None and len(self._sent_path) == len(current):
            if all(abs(a[0] - b[0]) < 0.10 and abs(a[1] - b[1]) < 0.10
                   for a, b in zip(self._sent_path, current)):
                return
        self._sent_path = list(current)
        self._client.send_goal_async(goal)

    def _warn(self, key, message, interval_s=10.0):
        now = time.monotonic()
        if now - self._warned_at.get(key, 0.0) >= interval_s:
            self.get_logger().warning(message)
            self._warned_at[key] = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RoadGoalNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
