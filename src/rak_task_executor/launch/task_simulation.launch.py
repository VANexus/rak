"""Start a deterministic software-only task lifecycle fixture."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("rak_task_executor")
    waypoints = os.path.join(share, "config", "waypoints_sim.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("goal_duration_sec", default_value="60.0"),
        Node(
            package="rak_task_executor",
            executable="fake_navigation",
            output="screen",
            parameters=[{
                "goal_duration_sec": ParameterValue(
                    LaunchConfiguration("goal_duration_sec"), value_type=float
                ),
            }],
        ),
        Node(
            package="rak_task_executor",
            executable="task_executor",
            output="screen",
            parameters=[{
                "waypoints_file": waypoints,
                "slow_planner": "local",
            }],
        ),
    ])
