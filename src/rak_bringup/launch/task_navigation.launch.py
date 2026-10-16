"""Combined text-task executor, SLAM/Nav2 and the selected motion backend.

Wires rak_task_executor (the task lifecycle layer) on top of the full Nav2
chain. motion_backend picks the final /cmd_vel adapter:
  - sim|stub   non-hardware adapters
  - hardware   removed (ADR-0016): the chassis is reached through
               chassis_bringup.launch.py -> mecanum_drive_controller over CAN,
               never through a second /cmd_vel adapter here.
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("rak_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub"],
            description="Final /cmd_vel adapter; the real chassis is reached via "
                        "chassis_bringup.launch.py (CAN), not this argument.",
        ),
        DeclareLaunchArgument("slow_planner", default_value="local",
                              choices=["local", "rai", "harness"]),
        DeclareLaunchArgument("llm_config_path", default_value=""),
        DeclareLaunchArgument("llm_vendor", default_value=""),
        DeclareLaunchArgument("harness", default_value="true",
                              choices=["true", "false"],
                              description="Start harness_node next to the executor "
                                          "(ADR-0010 arbitration seam)."),
        DeclareLaunchArgument("waypoints_file", default_value=""),
        DeclareLaunchArgument("lidar_z", default_value="0.1525"),  # 2026-10-14 实测
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                f"{navigation_share}/launch/nav2.launch.py"
            ),
            launch_arguments={
                "motion_backend": LaunchConfiguration("motion_backend"),
                "lidar_z": LaunchConfiguration("lidar_z"),
                "odom_source": LaunchConfiguration("odom_source"),
                "use_sim_time": "false",
            }.items(),
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
        Node(
            package="rak_harness",
            executable="harness_node",
            name="harness",
            output="screen",
            condition=IfCondition(LaunchConfiguration("harness")),
        ),
    ])
