"""demo.launch.py - full MoveIt2 demo (M3 + M4).

Spec section 7.6. Requires Jetson local :1 display (Xorg) for RViz + IK.

R8.1: MoveItConfigsBuilder in Jazzy MoveIt2 (v2.12.4) does NOT have
a `.kinematics()` method (renamed to `.robot_description_kinematics()`).
URDF/SRDF live in urdf/ (per spec section 7.1), so we pass explicit
file_path.

R8.2: MoveItConfigsBuilder in Jazzy v2.12.4 auto-loads 4 planning pipelines
(ompl, chomp, pilz, stomp) from /opt/ros/jazzy/share/moveit_configs_utils/
default_configs/. pilz_industrial_motion_planner crashes during init if
its pilz_cartesian_limits.yaml is missing or empty. Workaround: provide
a minimal pilz_cartesian_limits.yaml (already added to config/). The
"Planning plugin name is empty" error in some test runs was traced to
a transient config-loading race; with a clean build it is not present.

R10.1: static_transform_publisher world->base_link = identity so RViz
Fixed Frame='world' resolves before nav2/SLAM is running. Remove this
node once /map->base_link is live from slam_toolbox.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg = get_package_share_directory("rak_moveit_config")
    description = get_package_share_directory("rak_description")
    urdf_path = os.path.join(description, "urdf", "rak.urdf.xacro")
    srdf_path = os.path.join(pkg, "config", "rak.srdf")
    rviz_config = os.path.join(description, "config", "rviz_view.rviz")

    moveit_config = (
        MoveItConfigsBuilder("rak", package_name="rak_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .joint_limits(file_path=os.path.join(pkg, "config", "joint_limits.yaml"))
        .robot_description_kinematics(file_path=os.path.join(pkg, "config", "kinematics.yaml"))
        .planning_pipelines(
            pipelines=["ompl"],
            default_planning_pipeline="ompl",
        )
        .pilz_cartesian_limits(file_path=os.path.join(pkg, "config", "pilz_cartesian_limits.yaml"))
        .trajectory_execution(
            file_path=os.path.join(pkg, "config", "fake_controllers.yaml")
        )
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="log",
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(
                Command(["xacro ", urdf_path]), value_type=str
            ),
        }],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # R10.1: world->base_link fallback so RViz's Fixed Frame='world' resolves
    # without nav2/SLAM. Identity transform; chassis lives at world origin
    # in the demo scenario. Drop this node once /map->base_link is live.
    static_world_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_world_to_base_link",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        output="log",
    )

    return LaunchDescription([rsp, jsp, static_world_to_base, move_group, rviz])
