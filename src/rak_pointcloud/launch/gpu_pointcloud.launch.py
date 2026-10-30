"""Depth -> point cloud on the GPU.

Run alongside the camera with `enable_point_cloud:=false` so the SDK stops
building its own cloud:

    ros2 launch rak_bringup gemini335l.launch.py enable_point_cloud:=false
    ros2 launch rak_pointcloud gpu_pointcloud.launch.py

Measured on the Orin Nano (2026-10-07), 1280x800 input, this node against the
camera SDK's CPU cloud:

    decimation 1 (1.02M pts)  GPU 18.1%   SDK path ~8-11% (of the 15.2% camera container)
    decimation 4 (64k pts)    GPU  3.8%   SDK path 4.1%

**The GPU is not a CPU win here.** The back-projection is a rounding error next
to what the host pays to build, copy and publish a 16 MB message -- the GPU path
adds a 16 MB device->host copy on top of the same message cost. What actually
cut CPU was trimming the camera's streams and decimating the cloud (see
`rak_bringup/config/gemini335l.yaml`), not moving the maths.

Use this node when a consumer needs a **full-resolution** cloud and the CPU
budget allows it; otherwise the SDK path with `point_cloud_decimation_filter_factor`
is cheaper. Correctness is pinned by the analytic-plane check in
`docs/` and was verified against a synthetic 2 m plane at 1280x800.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from rak_orchestration.launch_helpers import managed


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/depth/image_raw",
                description="16-bit depth image (16UC1, millimetres).",
            ),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/depth/camera_info",
                description="Intrinsics for the depth stream.",
            ),
            DeclareLaunchArgument(
                "points_topic", default_value="/camera/depth/points",
                description="Cloud topic. Matches the SDK's name so consumers are unchanged.",
            ),
            DeclareLaunchArgument(
                "decimation", default_value="1",
                description="Keep every Nth pixel on each axis. 1 = full resolution.",
            ),
            DeclareLaunchArgument(
                "depth_scale", default_value="0.001",
                description="Metres per depth unit (Orbbec emits millimetres).",
            ),
            DeclareLaunchArgument("min_range_m", default_value="0.1"),
            DeclareLaunchArgument("max_range_m", default_value="10.0"),
            DeclareLaunchArgument(
                "log_every_n", default_value="0",
                description="Log per-frame latency every N frames; 0 disables.",
            ),
            DeclareLaunchArgument(
                "supervised", default_value="false",
                description="true = 由 system_manager 驱动生命周期（本文件不自启）",
            ),
            *managed(
                package="rak_pointcloud",
                executable="gpu_pointcloud_node",
                name="gpu_pointcloud",
                output="screen",
                parameters=[{
                    "depth_topic": LaunchConfiguration("depth_topic"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "points_topic": LaunchConfiguration("points_topic"),
                    "decimation": LaunchConfiguration("decimation"),
                    "depth_scale": LaunchConfiguration("depth_scale"),
                    "min_range_m": LaunchConfiguration("min_range_m"),
                    "max_range_m": LaunchConfiguration("max_range_m"),
                    "log_every_n": LaunchConfiguration("log_every_n"),
                }],
            ),
        ]
    )
