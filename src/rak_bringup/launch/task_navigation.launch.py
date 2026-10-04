"""Combined text-task executor, SLAM/Nav2 and explicitly simulated base output."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    navigation_share = get_package_share_directory("rak_navigation")
    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub"],
            description="Only non-hardware backends are available in this combined launch",
        ),
        DeclareLaunchArgument("slow_planner", default_value="local", choices=["local", "rai"]),
        DeclareLaunchArgument("llm_config_path", default_value=""),
        DeclareLaunchArgument("llm_vendor", default_value=""),
        DeclareLaunchArgument("waypoints_file", default_value=""),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
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
    ])
