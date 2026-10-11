"""Reuse Gemini bringup and optional existing GPU cloud; capture stays a bounded CLI."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    sensor = get_package_share_directory("rak_bringup")
    grasp = get_package_share_directory("rak_grasp_perception")
    cloud = get_package_share_directory("rak_pointcloud")
    return LaunchDescription([
        DeclareLaunchArgument("live_cloud", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(sensor, "launch", "gemini335l.launch.py")),
            launch_arguments={"config_file_path": os.path.join(grasp, "config", "gemini335l_grasp.yaml"),
                              "enable_point_cloud": "false"}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(cloud, "launch", "gpu_pointcloud.launch.py")),
            condition=IfCondition(LaunchConfiguration("live_cloud"))),
    ])
