"""Combined text-task executor, SLAM/Nav2 and the selected motion backend.

Wires rak_task_executor (the task lifecycle layer) on top of the full Nav2
chain. motion_backend picks the final /cmd_vel adapter:
  - sim|stub   non-hardware
  - hardware   keeps the Nav2 layer endpoint-free and starts the F407 UART
               adapter here at the robot bringup layer.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("rak_navigation")
    bringup_share = get_package_share_directory("rak_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub", "hardware"],
            description="Final /cmd_vel adapter; hardware drives the F407 base UART.",
        ),
        DeclareLaunchArgument("slow_planner", default_value="local", choices=["local", "rai"]),
        DeclareLaunchArgument("llm_config_path", default_value=""),
        DeclareLaunchArgument("llm_vendor", default_value=""),
        DeclareLaunchArgument("waypoints_file", default_value=""),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        # Hardware-backend passthrough (only consumed when motion_backend:=hardware).
        DeclareLaunchArgument("port", default_value="/dev/wheel_motor"),
        DeclareLaunchArgument("baud", default_value="921600"),
        DeclareLaunchArgument("hardware_enable", default_value="false"),
        DeclareLaunchArgument("max_vx_m_s", default_value="0.0"),
        DeclareLaunchArgument("max_vy_m_s", default_value="0.0"),
        DeclareLaunchArgument("max_wz_rad_s", default_value="0.0"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{navigation_share}/launch/nav2.launch.py"
            ),
            launch_arguments={
                "motion_backend": PythonExpression([
                    "'none' if '",
                    LaunchConfiguration("motion_backend"),
                    "' == 'hardware' else '",
                    LaunchConfiguration("motion_backend"),
                    "'",
                ]),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "odom_source": LaunchConfiguration("odom_source"),
                "use_sim_time": "false",
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{bringup_share}/launch/base_uart.launch.py"
            ),
            launch_arguments={
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
                "hardware_enable": LaunchConfiguration("hardware_enable"),
                "max_vx_m_s": LaunchConfiguration("max_vx_m_s"),
                "max_vy_m_s": LaunchConfiguration("max_vy_m_s"),
                "max_wz_rad_s": LaunchConfiguration("max_wz_rad_s"),
            }.items(),
            condition=IfCondition(
                PythonExpression([
                    "'", LaunchConfiguration("motion_backend"), "' == 'hardware'",
                ])
            ),
        ),
        Node(
            package="rak_task_executor",
            executable="task_executor",
            name="task_executor",
            output="screen",
            parameters=[{
                "slow_planner": LaunchConfiguration("slow_planner"),
                "llm_config_path": LaunchConfiguration("llm_config_path"),
                "llm_vendor": LaunchConfiguration("llm_vendor"),
                "waypoints_file": LaunchConfiguration("waypoints_file"),
            }],
        ),
    ])
