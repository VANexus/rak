"""Launch stereo capture with the existing driver defaults and topic names.

CaptureNode 是 LifecycleNode（ADR-0020）：CUDA 上下文与 V4L2 设备在 configure 建立，
采集线程在 activate 才启动 —— 顺序原来还是反的（先 cudaMalloc 后 init_cuda）。
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
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("left_camera_info_url", default_value=""),
        DeclareLaunchArgument("right_camera_info_url", default_value=""),
        *managed(
            package="rak_stereo_driver",
            executable="stereo_node",
            name="eye_capture",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"),
                "left_camera_info_url": LaunchConfiguration("left_camera_info_url"),
                "right_camera_info_url": LaunchConfiguration("right_camera_info_url"),
            }],
        ),
    ])
