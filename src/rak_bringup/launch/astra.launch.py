"""Launch Astra image streams; installation TF must come from measured calibration.

AstraBridgeNode 是 LifecycleNode（ADR-0020）：资源（设备/流配置）在 configure 打开、
开流在 activate。`supervised:=true` 时由 system_manager 驱动，本文件不自启。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from rak_orchestration.launch_helpers import managed


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 由 system_manager 驱动生命周期（本文件不自启）"),
        DeclareLaunchArgument("color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("depth_frame_id", default_value="camera_depth_frame"),
        *managed(
            package="rak_astra_driver",
            executable="astra_node",
            name="astra_bridge_node",
            output="screen",
            parameters=[{
                "color_frame_id": LaunchConfiguration("color_frame_id"),
                "depth_frame_id": LaunchConfiguration("depth_frame_id"),
            }],
        ),
    ])
