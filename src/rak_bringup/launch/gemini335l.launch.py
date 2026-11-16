"""Launch the Orbbec Gemini 335L through Orbbec's SDK v2 ROS 2 wrapper.

The upstream launch (gemini_330_series.launch.py) owns the device; this adapter
only picks a stream profile and forwards the switches we care about.
"""

from pathlib import Path

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#: profile -> stream budget. wifi is the default because the robot streams to a
#: developer machine over 5 GHz WiFi (~4 MB/s measured); full needs a cable or a
#: local display, 1280x800 RGB alone is ~92 MB/s of copies.
_PROFILES = {
    "wifi": "gemini335l.yaml",
    "full": "gemini335l_full.yaml",
}

#: ir -> (enable_left_ir, enable_right_ir). Off by default: two more 16-bit
#: streams over WiFi buys nothing for navigation, the depth image already has
#: the same geometry.
_IR = {
    "none": ("false", "false"),
    "left": ("true", "false"),
    "right": ("false", "true"),
    "both": ("true", "true"),
}


def _include_orbbec_driver(context):
    try:
        driver_launch = (
            Path(get_package_share_directory("orbbec_camera"))
            / "launch"
            / "gemini_330_series.launch.py"
        )
    except PackageNotFoundError as exc:
        # 开发机（lyrical）上没有 ros-lyrical-orbbec-camera：这条 backend 只能在
        # orin 上跑。给一句人话，而不是 ament_index 的堆栈。
        raise RuntimeError(
            "orbbec_camera is not installed in this ROS environment; the Gemini "
            "backend runs on the Jetson (ros-jazzy-orbbec-camera). Use "
            "camera:=astra on this machine, or point ROS at the Jetson."
        ) from exc
    bringup_share = Path(get_package_share_directory("rak_bringup"))
    camera_name = LaunchConfiguration("camera_name").perform(context)
    serial_number = LaunchConfiguration("serial_number").perform(context)
    enable_point_cloud = LaunchConfiguration("enable_point_cloud").perform(context)
    config_file_path = LaunchConfiguration("config_file_path").perform(context)
    profile = LaunchConfiguration("profile").perform(context)
    ir = LaunchConfiguration("ir").perform(context)
    if profile not in _PROFILES:
        raise ValueError(f"unknown gemini335l profile '{profile}', expected one of {sorted(_PROFILES)}")
    if ir not in _IR:
        raise ValueError(f"unknown ir mode '{ir}', expected one of {sorted(_IR)}")
    if not config_file_path:
        config_file_path = str(bringup_share / "config" / _PROFILES[profile])
    enable_left_ir, enable_right_ir = _IR[ir]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_launch)),
            launch_arguments={
                "camera_name": camera_name,
                "serial_number": serial_number,
                "enable_point_cloud": enable_point_cloud,
                "enable_colored_point_cloud": "false",
                "enable_left_ir": enable_left_ir,
                "enable_right_ir": enable_right_ir,
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
                "profile",
                default_value="wifi",
                choices=sorted(_PROFILES),
                description="wifi = 424x240 depth@10 + color@5 (cross-host RViz); "
                            "full = 1280x800@30 (local display or cable).",
            ),
            DeclareLaunchArgument(
                "ir",
                default_value="none",
                choices=sorted(_IR),
                description="Enable left/right IR streams (each is another 16-bit image).",
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
