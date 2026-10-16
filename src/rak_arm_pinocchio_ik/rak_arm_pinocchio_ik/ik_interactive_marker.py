"""Self-contained IK interactive marker (drag-to-IK) node.

Why a custom node instead of MoveIt's robot_interaction server
(which on Jazzy fails to start due to the `rviz_ssp_<hash>` namespace
bug — see xrak-arm-pinocchio-ik-2026-10-09 memory K4):

  1. Publishes a 6-DOF drag handle on the `tool0` TF frame using
     ROS 2's apt interactive_markers server.
  2. On drag, composes the handle pose (which is relative to `tool0`)
     with the current tool0 pose in base_link (computed via Pinocchio
     forward kinematics from the current joint state) to obtain the
     desired end-effector pose in base_link.
  3. Calls the move_group `/compute_ik` service to get a joint solution.
  4. Publishes that joint state to `/joint_states`, replacing
     `fake_joint_controllers` so RViz actually follows the drag.

This node is the missing fifth thing that turned the IK service into
a working interactive UX on Jazzy.

Owner: xrak.
"""

from __future__ import annotations

import math

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from geometry_msgs.msg import Pose
from interactive_markers import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from .ik_service import PinocchioArmKinematics, load_urdf


def _quat_to_rot(q) -> np.ndarray:
    """geometry_msgs/Quaternion -> 3x3 rotation matrix (inline; no tf_transformations)."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _rot_to_quat(r: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 rotation matrix -> (x, y, z, w). Shepperd's method (numerically stable)."""
    m00, m01, m02 = float(r[0, 0]), float(r[0, 1]), float(r[0, 2])
    m10, m11, m12 = float(r[1, 0]), float(r[1, 1]), float(r[1, 2])
    m20, m21, m22 = float(r[2, 0]), float(r[2, 1]), float(r[2, 2])
    trace = m00 + m11 + m22
    if trace > 0.0:
        s = 0.5 / (trace + 1.0) ** 0.5
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * (1.0 + m00 - m11 - m22) ** 0.5
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * (1.0 + m11 - m00 - m22) ** 0.5
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * (1.0 + m22 - m00 - m11) ** 0.5
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s
    return x, y, z, w


class IkMarkerNode(Node):
    GROUP = "arm"
    TIP = "tool0"
    JOINT_NAMES = (
        "j1_base",
        "j2_shoulder",
        "j3_elbow",
        "j4_forearm",
        "j5_wrist_pitch",
        "j6_wrist_roll",
    )

    def __init__(self) -> None:
        super().__init__("ik_interactive_marker")

        # IK service client
        self._ik = self.create_client(GetPositionIK, "/compute_ik")
        self.get_logger().info("waiting for /compute_ik ...")
        while not self._ik.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                raise SystemExit(1)
        self.get_logger().info("/compute_ik available")

        # Joint state subscriber (track current q for FK composition)
        self._current_q = None
        self.create_subscription(JointState, "/joint_states", self._on_joint_state, 30)

        # Joint state publisher (replaces fake_joint_controllers)
        # TRANSIENT_LOCAL durability so any subscriber (RViz's TF display,
        # ros2 topic echo, move_group) that arrives after init still sees
        # the latest joint state and the tool0 TF chain stays alive.
        self._js_pub = self.create_publisher(
            JointState,
            "/joint_states",
            qos_profile=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Periodic republish at 10 Hz keeps the TF chain animated and ensures
        # /joint_states consumers that subscribed late still see updates.
        self._rep_timer = self.create_timer(0.1, self._publish_current_q)

        # Pinocchio kinematics for FK on the live joint state
        urdf_path = (
            get_package_share_directory("rak_description")
            + "/urdf/six_axis_sim.urdf.xacro"
        )
        self._arm = PinocchioArmKinematics(
            load_urdf(urdf_path), joint_names=list(self.JOINT_NAMES), tip_frame=self.TIP
        )

        # Interactive marker: 6-DOF handle on tool0 (rotation 3D + translation 3D
        # via the stock MOVE_3D / ROTATE_3D control modes — no need to manually
        # build 6 arrows/rings).
        # The QoS profile is RELIABLE+TRANSIENT_LOCAL: RViz2's
        # InteractiveMarkers display requests TRANSIENT_LOCAL durability so
        # newly-connected viewers get the current marker state. Default is
        # VOLATILE which fails the DDS compatibility check and silently drops
        # RViz's subscription (no warning printed by either side).
        self._server = InteractiveMarkerServer(
            self,
            "/ik_marker_topic",
            update_pub_qos=QoSProfile(
                depth=10,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.TIP
        int_marker.name = "ik_6dof"
        int_marker.scale = 0.20
        int_marker.pose = Pose()  # identity: handle sits at tool0 itself

        move_ctrl = InteractiveMarkerControl()
        move_ctrl.name = "translate"
        move_ctrl.interaction_mode = InteractiveMarkerControl.MOVE_3D
        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.03
        sphere.color.r = 1.0
        sphere.color.g = 0.6
        sphere.color.a = 0.8
        move_ctrl.markers.append(sphere)
        int_marker.controls.append(move_ctrl)

        rot_ctrl = InteractiveMarkerControl()
        rot_ctrl.name = "rotate"
        rot_ctrl.interaction_mode = InteractiveMarkerControl.ROTATE_3D
        int_marker.controls.append(rot_ctrl)

        self._server.insert(int_marker, feedback_callback=self._on_feedback)
        self._server.applyChanges()
        self.get_logger().info("drag handle published on tool0 (topic /ik_marker_topic)")

        # Publish a single /joint_states=0 message so robot_state_publisher builds
        # the per-joint TF chain (base_link -> ... -> tool0). Without this,
        # the tool0 frame does not exist on /tf and RViz cannot place the
        # interactive marker correctly.
        init = JointState()
        init.header.stamp = self.get_clock().now().to_msg()
        init.name = list(self.JOINT_NAMES)
        init.position = [0.0] * self._arm.n_joints
        self._js_pub.publish(init)
        self._current_q = np.zeros(self._arm.n_joints, dtype=float)
        self.get_logger().info("published initial 0 joint_states to seed TF chain")

    def _on_joint_state(self, msg: JointState) -> None:
        q = np.full(self._arm.n_joints, np.nan, dtype=float)
        for n, p in zip(msg.name, msg.position):
            if n in self._arm.joint_names:
                q[self._arm.joint_names.index(n)] = float(p)
        if np.isfinite(q).all():
            self._current_q = q

    def _on_feedback(self, fb) -> None:
        if fb.event_type not in (fb.MOUSE_UP, fb.POSE_UPDATE):
            return
        if self._current_q is None:
            self.get_logger().warn("no joint state yet; skipping drag")
            return

        # Compose: target_tool0_in_base = base_T_tool0 * marker_pose
        p_cur, r_cur = self._arm.forward_kinematics(self._current_q)
        r_m = _quat_to_rot(fb.pose.orientation)
        target_p = p_cur + r_cur @ np.array(
            [fb.pose.position.x, fb.pose.position.y, fb.pose.position.z]
        )
        target_r = r_cur @ r_m

        # Solve IK via move_group
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.GROUP
        req.ik_request.pose_stamped.header.frame_id = "base_link"
        req.ik_request.pose_stamped.pose.position.x = float(target_p[0])
        req.ik_request.pose_stamped.pose.position.y = float(target_p[1])
        req.ik_request.pose_stamped.pose.position.z = float(target_p[2])
        qx, qy, qz, qw = _rot_to_quat(target_r)
        req.ik_request.pose_stamped.pose.orientation.x = qx
        req.ik_request.pose_stamped.pose.orientation.y = qy
        req.ik_request.pose_stamped.pose.orientation.z = qz
        req.ik_request.pose_stamped.pose.orientation.w = qw
        req.ik_request.timeout.sec = 1
        req.ik_request.robot_state.joint_state.name = list(self.JOINT_NAMES)
        req.ik_request.robot_state.joint_state.position = [float(v) for v in self._current_q]

        fut = self._ik.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done() or fut.result() is None:
            self.get_logger().warn("IK call timed out / no response")
            return

        resp = fut.result()
        if resp.error_code.val != 1:  # SUCCESS = 1
            self.get_logger().warn(
                f"IK did not converge (code={resp.error_code.val}); marker held in place"
            )
            return

        # Publish new joint state to /joint_states
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = [float(v) for v in resp.solution.joint_state.position]
        self._js_pub.publish(msg)
        self._current_q = np.array(msg.position)


    def _publish_current_q(self) -> None:
        if self._current_q is None:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.JOINT_NAMES)
        msg.position = [float(v) for v in self._current_q]
        self._js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(IkMarkerNode())
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
