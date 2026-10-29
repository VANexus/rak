"""Six-axis sim with Pinocchio IK + self-contained drag handle.

Same bringup as six_axis_sim.launch.py except:
  * uses `rak_arm_pinocchio_kinematics.yaml` (srv_kinematics_plugin +
    rak_arm_pinocchio_ik service) instead of stock KDL
  * starts pinocchio_ik_service + ik_marker_node ahead of move_group
  * drops fake_joint_controllers — ik_marker_node publishes /joint_states
    from drag feedback. MoveIt's robot_interaction server is intentionally
    skipped (it does not start on Jazzy due to the namespace bug).

RViz is launched separately — this file only starts the MoveIt side so
the RViz window on :1 can attach to the same ROS graph.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("rak_moveit_config")
    description_share = get_package_share_directory("rak_description")
    urdf = os.path.join(description_share, "urdf", "six_axis_sim.urdf.xacro")
    srdf = os.path.join(moveit_share, "config", "six_axis_sim.srdf")
    controllers_path = os.path.join(
        moveit_share, "config", "six_axis_sim_controllers.yaml"
    )
    limits_path = os.path.join(
        moveit_share, "config", "six_axis_sim_joint_limits.yaml"
    )
    kinematics_path = os.path.join(
        moveit_share, "config", "rak_arm_pinocchio_kinematics.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder("rak_six_axis_sim", package_name="rak_moveit_config")
        .robot_description(file_path=urdf)
        .robot_description_semantic(file_path=srdf)
        .joint_limits(file_path=limits_path)
        .robot_description_kinematics(file_path=kinematics_path)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .trajectory_execution(file_path=controllers_path)
        .to_moveit_configs()
        .to_dict()
    )
    moveit_config["moveit_controller_manager"] = (
        "moveit_simple_controller_manager/MoveItSimpleControllerManager"
    )
    with open(controllers_path, encoding="utf-8") as open_file:
        moveit_config["moveit_simple_controller_manager"] = yaml.safe_load(open_file)

    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_world_tf",
            default_value="true",
            description="Publish static world->base_link.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="sim_world_to_base",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "0", "--pitch", "0", "--yaw", "0",
                "--frame-id", "world", "--child-frame-id", "base_link",
            ],
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_world_tf")),
        ),
        Node(
            package="rak_arm_pinocchio_ik",
            executable="ik_service_node",
            name="pinocchio_ik_service",
            output="screen",
            parameters=[{
                "urdf_path": urdf,
                "joint_names": [
                    "j1_base",
                    "j2_shoulder",
                    "j3_elbow",
                    "j4_forearm",
                    "j5_wrist_pitch",
                    "j6_wrist_roll",
                ],
                "tip_frame": "tool0",
                "group_name": "arm",
            }],
        ),
        # Self-contained 6-DOF drag handle on tool0 + IK feedback loop + /joint_states
        # publish. Bypasses MoveIt's robot_interaction server which fails on Jazzy
        # due to the `rviz_ssp_<hash>` namespace bug.
        Node(
            package="rak_arm_pinocchio_ik",
            executable="ik_marker_node",
            name="ik_marker",
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config],
            output="screen",
        ),
    ])
