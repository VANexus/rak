"""Launch the Orbbec Gemini 335L through Orbbec's SDK v2 ROS 2 wrapper."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _include_orbbec_driver(context):
    driver_launch = (
        Path(get_package_share_directory("orbbec_camera"))
        / "launch"
        / "gemini_330_series.launch.py"
    )
    bringup_share = Path(get_package_share_directory("rak_sensor_bringup"))
    camera_name = LaunchConfiguration("camera_name").perform(context)
    serial_number = LaunchConfiguration("serial_number").perform(context)
    enable_point_cloud = LaunchConfiguration("enable_point_cloud").perform(context)
    config_file_path = LaunchConfiguration("config_file_path").perform(context)
    if not config_file_path:
        config_file_path = str(bringup_share / "config" / "gemini335l.yaml")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_launch)),
            launch_arguments={
                "camera_name": camera_name,
                "serial_number": serial_number,
                "enable_point_cloud": enable_point_cloud,
                "enable_colored_point_cloud": "false",
                "config_file_path": config_file_path,
            }.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera_name",
                default_value="camera",
                description="ROS namespace and frame prefix used by the camera driver.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="CP2G853000G0",
                description="Gemini 335L serial number for deterministic device selection.",
            ),
            DeclareLaunchArgument(
                "enable_point_cloud",
                default_value="false",
                description="Enable SDK point-cloud publication when a consumer needs it.",
            ),
            DeclareLaunchArgument(
                "config_file_path",
                default_value="",
                description="Override the packaged Gemini stream configuration YAML.",
            ),
            OpaqueFunction(function=_include_orbbec_driver),
        ]
    )
