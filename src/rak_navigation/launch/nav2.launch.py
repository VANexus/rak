"""SLAM + omnidirectional Nav2 with optional local motion-test adapters.

Velocity path: MPPI / behavior -> velocity smoother -> collision monitor ->
the selected final-Twist adapter on /cmd_vel. ``none`` leaves the endpoint to
an outer bringup layer, which can compose a hardware driver without reversing
the package dependency direction:

  - sim        cmd_vel_sim      (sim-only /base/telemetry mirror)
  - stub       cmd_vel_stub     (Twist relay -> /cmd_vel_raw)
  - none       no adapter in this launch
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("rak_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    nav2_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    backend = LaunchConfiguration("motion_backend")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub", "none"],
            description="Final /cmd_vel adapter; none leaves it to an outer bringup.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "slam.launch.py")),
            launch_arguments={
                "lidar_z": LaunchConfiguration("lidar_z"),
                "odom_source": LaunchConfiguration("odom_source"),
            }.items(),
        ),
        Node(
            package="rak_motion_stub",
            executable="cmd_vel_sim",
            name="cmd_vel_sim",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "telemetry_topic": "/base/telemetry",
            }],
            condition=IfCondition(EqualsSubstitution(backend, "sim")),
        ),
        Node(
            package="rak_motion_stub",
            executable="cmd_vel_stub",
            name="cmd_vel_stub",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "raw_topic": "/cmd_vel_raw",
                "log_every_n": 5,
            }],
            condition=IfCondition(EqualsSubstitution(backend, "stub")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_share, "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "params_file": nav2_params,
                "use_sim_time": use_sim_time,
                "autostart": "True",
            }.items(),
        ),
    ])
