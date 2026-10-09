"""Launch the six-axis software fixture and its isolated fake action servers."""

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
        moveit_share, "config", "six_axis_sim_kinematics.yaml"
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
    with open(controllers_path, encoding="utf-8") as stream:
        moveit_config["moveit_simple_controller_manager"] = yaml.safe_load(stream)

    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)
    return LaunchDescription([
        # The arm URDF's root link IS `base_link`. Standalone, the fixture needs a
        # `world -> base_link` to give TF a root; on the mobile robot that frame belongs
        # to SLAM/Nav2 (`map -> odom -> base_link`) and a second parent would make the
        # tree ambiguous. Set false when composing with the base.
        DeclareLaunchArgument(
            "publish_world_tf",
            default_value="true",
            description="Publish static world->base_link. False when a mobile base owns base_link.",
        ),
        # RViz needs an X display. This box runs headless by default (gdm masked), where
        # rviz2 aborts on startup and takes the launch down with it.
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Start RViz. Set false headless (see scripts/rak-mode.sh desktop).",
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
            package="rak_arm_simulation",
            executable="fake_joint_controllers",
            output="screen",
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            parameters=[moveit_config],
            output="screen",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=[
                "-d",
                os.path.join(description_share, "config", "rviz_view.rviz"),
            ],
            output="log",
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
