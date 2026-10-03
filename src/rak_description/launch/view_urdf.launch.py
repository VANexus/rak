"""view_urdf.launch.py - headless URDF + joint_state visualization.

Spec section 7.6. No RViz; safe to run over plain SSH for M1/M2 validation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rclpy.parameter import Parameter


def generate_launch_description():
    pkg = get_package_share_directory("rak_description")
    urdf_path = os.path.join(pkg, "urdf", "rak.urdf.xacro")

    # Command joins args WITHOUT spaces; trailing space after 'xacro '.
    # ParameterValue(value, value_type=str) tells launch to pass as string,
    # not YAML-parse it.
    robot_description = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    jsp = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return LaunchDescription([rsp, jsp])