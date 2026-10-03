"""display.launch.py - view_urdf + RViz (M2 visualization).

Spec section 7.6. Requires Jetson local :1 display or X-forwarding.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("boot")
    urdf_path = os.path.join(pkg, "urdf", "rak.urdf.xacro")
    rviz_config = os.path.join(pkg, "config", "rviz_view.rviz")

    robot_description = Command(["xacro ", urdf_path])

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
        parameters=[{"use_sim_time": False}],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([rsp, jsp, rviz])