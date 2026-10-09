"""Launch exactly one selectable RGB-D camera backend."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _launch_selected_camera(context):
    camera = LaunchConfiguration("camera").perform(context)
    if camera == "astra":
        launch_file = "astra.launch.py"
        launch_arguments = {}
    elif camera == "gemini335l":
        launch_file = "gemini335l.launch.py"
        launch_arguments = {
            "serial_number": LaunchConfiguration("serial_number").perform(context),
        }
    else:
        raise ValueError(f"unsupported depth camera backend: {camera}")

    bringup_share = Path(get_package_share_directory("rak_bringup"))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(bringup_share / "launch" / launch_file)
            ),
            launch_arguments=launch_arguments.items(),
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "camera",
                default_value="gemini335l",
                choices=["gemini335l", "astra"],
                description="Select exactly one depth camera driver.",
            ),
            DeclareLaunchArgument(
                "serial_number",
                default_value="CP2G853000G0",
                description="Gemini serial number; ignored when camera:=astra.",
            ),
            OpaqueFunction(function=_launch_selected_camera),
        ]
    )
