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
        launch_arguments = {
            # 跨机 RViz：320x240 的图像+点云会打满 WiFi，远程可视化时降到 160。
            "depth_max_width": LaunchConfiguration("depth_max_width").perform(context),
        }
    elif camera == "gemini335l":
        launch_file = "gemini335l.launch.py"
        launch_arguments = {
            "serial_number": LaunchConfiguration("serial_number").perform(context),
            # wifi = 424x240 depth@10 + color@5（跨机 RViz）；full = 1280x800@30。
            "profile": LaunchConfiguration("gemini_profile").perform(context),
            "ir": LaunchConfiguration("gemini_ir").perform(context),
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
            DeclareLaunchArgument(
                "depth_max_width",
                default_value="320",
                description="Astra depth width cap (0 = no cap); lower for remote RViz.",
            ),
            DeclareLaunchArgument(
                "gemini_profile",
                default_value="wifi",
                choices=["wifi", "full"],
                description="Gemini 335L stream budget: wifi (424x240 depth@10, "
                            "color@5) or full (1280x800@30, local display only).",
            ),
            DeclareLaunchArgument(
                "gemini_ir",
                default_value="none",
                choices=["none", "left", "right", "both"],
                description="Gemini 335L IR streams; each is another 16-bit image.",
            ),
            OpaqueFunction(function=_launch_selected_camera),
        ]
    )
