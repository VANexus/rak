"""Launch Astra image streams; installation TF must come from measured calibration.

AstraBridgeNode 是 LifecycleNode（ADR-0020）：资源（设备/流配置）在 configure 打开、
开流在 activate。`supervised:=true` 时由 system_manager 驱动，本文件不自启。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from rak_orchestration.launch_helpers import managed


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 由 system_manager 驱动生命周期（本文件不自启）"),
        DeclareLaunchArgument("color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("depth_frame_id", default_value="camera_depth_frame"),
        # 跨机 RViz 用：320x240@30 的 16UC1 图像本身就是 ~4.5 MB/s，实测会打满
        # 本机到 orin 的 WiFi（~4 MB/s），把 /scan、/imu 全饿死。远程可视化时传
        # depth_max_width:=160（图像降到 ~1.1 MB/s）。本机消费者可保持默认 320。
        DeclareLaunchArgument(
            "depth_max_width", default_value="320",
            description="cap depth stream width (0 = no cap); lower it for remote RViz"),
        *managed(
            package="rak_astra_driver",
            executable="astra_node",
            name="astra_bridge_node",
            output="screen",
            parameters=[{
                "color_frame_id": LaunchConfiguration("color_frame_id"),
                "depth_frame_id": LaunchConfiguration("depth_frame_id"),
                "depth_max_width": ParameterValue(
                    LaunchConfiguration("depth_max_width"), value_type=int),
            }],
        ),
    ])
