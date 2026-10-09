"""Launch the Pinocchio IK service alongside the rest of MoveIt.

Run after the six-axis fixture launch (which already starts move_group +
fake_joint_controllers + robot_state_publisher). This launch only adds the
IK service node; srv_kinematics_plugin inside move_group will discover it
via the `/solve_ik` and `/get_ik_solver_info` services declared in
`rak_arm_pinocchio_kinematics.yaml`.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rak_arm_pinocchio_ik",
            executable="ik_service_node",
            name="pinocchio_ik_service",
            output="screen",
            parameters=[{
                "urdf_path": "",
                "joint_names": [
                    "base_yaw_joint",
                    "shoulder_pitch_joint",
                    "elbow_pitch_joint",
                    "wrist_yaw_joint",
                    "wrist_pitch_joint",
                    "wrist_roll_joint",
                ],
                "tip_frame": "tool0",
                "group_name": "arm",
            }],
        ),
    ])
