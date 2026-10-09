"""rclpy service node exposing Pinocchio IK to MoveIt.

Wires the single service consumed by `srv_kinematics_plugin/SrvKinematicsPlugin`:

    /solve_ik       moveit_msgs/srv/GetPositionIK

The plugin's default `kinematics_solver_service_name` is `solve_ik`; we
keep that default so a fresh `kinematics.yaml` needs only the plugin class
name. Solver info (joint names, link names, limits) is read by the plugin
from MoveIt's standard URDF/SRDF chain — there is no separate service for
it (the `GetKinematicSolverInfo` type that existed in MoveIt 1 was never
shipped in moveit_msgs for ROS 2; srv_kinematics_plugin stores the info
locally instead). See the architecture note in setup.py for why we use
the service plugin instead of writing a Python KinematicsBase subclass
(MoveIt 2 Jazzy does not expose that class to Python).
"""

from __future__ import annotations

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node

from .ik_service import PinocchioArmKinematics, load_urdf


class PinocchioIkService(Node):
    """Bridge: MoveIt GetPositionIK -> pinocchio DLS -> MoveIt RobotState."""

    # `six_axis_sim.srdf` (and the real arm SRDF) name the planning group `arm`.
    # srv_kinematics_plugin forwards the planning group's joint/tip names from
    # the plugin parameter block, but the request still carries a group_name
    # we use as a sanity guard.
    GROUP_NAME = "arm"

    def __init__(self) -> None:
        super().__init__("pinocchio_ik_service")

        self.declare_parameter("urdf_path", "")
        self.declare_parameter("joint_names", [
            "base_yaw_joint",
            "shoulder_pitch_joint",
            "elbow_pitch_joint",
            "wrist_yaw_joint",
            "wrist_pitch_joint",
            "wrist_roll_joint",
        ])
        self.declare_parameter("tip_frame", "tool0")
        self.declare_parameter("group_name", self.GROUP_NAME)

        urdf_path = (
            self.get_parameter("urdf_path").get_parameter_value().string_value
        )
        if not urdf_path:
            share = get_package_share_directory("rak_arm_pinocchio_ik")
            urdf_path = share + "/config/six_axis_sim.urdf"

        joint_names = list(
            self.get_parameter("joint_names").get_parameter_value().string_array_value
        )
        tip_frame = self.get_parameter("tip_frame").get_parameter_value().string_value

        self.get_logger().info(
            f"loading URDF: {urdf_path} (joints={joint_names}, tip={tip_frame})"
        )
        self._arm = PinocchioArmKinematics(
            load_urdf(urdf_path),
            joint_names=joint_names,
            tip_frame=tip_frame,
        )
        self._expected_group = self.get_parameter("group_name").get_parameter_value().string_value

        self._ik = self.create_service(
            GetPositionIK, "/solve_ik", self._on_solve_ik
        )
        self.get_logger().info("pinocchio IK service ready")

    def _on_solve_ik(self, request: GetPositionIK.Request, response: GetPositionIK.Response):
        req = request.ik_request
        if req.group_name and req.group_name != self._expected_group:
            response.error_code.val = MoveItErrorCodes.INVALID_GROUP_NAME
            return response

        pose: PoseStamped = req.pose_stamped
        target_p, target_r = _pose_to_p_r(pose.pose)

        seed = None
        if req.robot_state and req.robot_state.joint_state:
            seed = self._extract_seed(req.robot_state.joint_state)

        # MoveIt sometimes asks for position-only IK by setting the rotation
        # parts to NaN. Detect that and ignore orientation.
        position_only = _pose_is_position_only(pose.pose)
        q, ok = self._arm.solve_ik(
            target_p, target_r, seed=seed, position_only=position_only
        )

        sol = response.solution
        sol.joint_state.name = list(self._arm.joint_names)
        sol.joint_state.position = [float(v) for v in q]
        sol.joint_state.header.frame_id = req.pose_stamped.header.frame_id or self._arm.tip_frame
        sol.joint_state.header.stamp = self.get_clock().now().to_msg()

        response.error_code.val = (
            MoveItErrorCodes.SUCCESS if ok else MoveItErrorCodes.NO_IK_SOLUTION
        )
        return response

    def _extract_seed(self, joint_state):
        seed = np.full(self._arm.n_joints, np.nan, dtype=float)
        for name, pos in zip(joint_state.name, joint_state.position):
            if name in self._arm.joint_names:
                seed[self._arm.joint_names.index(name)] = pos
        if not np.isfinite(seed).all():
            return None
        return seed


# --- small helpers ---------------------------------------------------------

def _pose_to_p_r(pose: Pose):
    p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    q = pose.orientation
    return p, _quat_to_rot(q.x, q.y, q.z, q.w)


def _quat_to_rot(x, y, z, w):
    import numpy as np
    n = (x * x + y * y + z * z + w * w) ** 0.5
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def _pose_is_position_only(pose: Pose) -> bool:
    """MoveIt marks position-only IK by setting quaternion components to NaN."""
    import math
    for v in (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w):
        if isinstance(v, float) and math.isnan(v):
            return True
    return False


def main(args=None):
    rclpy.init(args=args)
    node = PinocchioIkService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
