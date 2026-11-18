"""Front camera: single UVC mono camera (ARC 05a3:9230, "AX USB2.0") via v4l2_camera.

Measured on this robot (2026-11-15):
  * nodes /dev/video0 (video1 is the UVC metadata node, no formats)
  * MJPG does 1280x720@31 (and 1920x1080@30); YUYV drops to 9 fps at 720p but does
    30 fps at 640x480.
  * `v4l2_camera` 0.7.3 cannot decode MJPG ("Current pixel format is not supported yet:
    MJPG"), so it is limited to YUYV 640x480@30 here.
  * `usb_cam` (the other official UVC driver) DOES do MJPG via `mjpeg2rgb`.
    Measured at 1280x720@30: 29.7 Hz, 55.9% of one core, RSS 94 MB. v4l2_camera at
    640x480@30: 29.6 Hz, 18.8%, RSS 68 MB.
  * DEFAULT IS usb_cam 1280x720. Reason: the ChArUco board fills a 640x480 frame at
    working distance, so only ~6 views are usable there and the calibration is
    under-constrained. At 1280x720 the same board gives 24 views / 0.40 px rms.
    The intrinsics in camera_info are published verbatim -- they are NOT rescaled -- so
    the calibration resolution and the driver resolution must match. Switching to
    `driver:=v4l2` (640x480) requires recalibrating at 640x480.
  * The camera is mounted upright (re-mounted 2026-11-16). Frames are published raw and
    calibrated raw; nothing downstream rotates or flips them.
  * it is a MONO camera: the frame fills the whole sensor, it is NOT a
    side-by-side stereo pair. Do not point `rak_stereo_driver` at it -- that
    driver splits SBS frames into two eyes and would emit two half-images.
  * the stereo pair stays reserved for the arm's eye-in-hand (vision_nav_v1.md §5A.4).

`camera_info` needs intrinsics before AprilTag PnP or rtabmap produce metric poses:
run `ros2 run camera_calibration cameracalibrator` once, save the yaml, then pass
`camera_info_url:=file://$HOME/rak/data/calib/front_camera.yaml`（把 $HOME 展开成实际路径）。
"""
import os

from ament_index_python.packages import get_package_share_directory  # noqa: F401
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    pixel_format = LaunchConfiguration("pixel_format")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_sim_time = LaunchConfiguration("use_sim_time")
    driver = LaunchConfiguration("driver")
    camera_name = LaunchConfiguration("camera_name")

    return LaunchDescription([
        DeclareLaunchArgument(
            "driver",
            default_value="usb_cam",
            choices=["usb_cam", "v4l2"],
            description="usb_cam: MJPG 1280x720@30, 55.9% CPU (default, matches the "
                        "calibration). v4l2: YUYV 640x480@30, 18.8% CPU (needs its own "
                        "calibration).",
        ),
        DeclareLaunchArgument("video_device", default_value="/dev/video0"),
        # v4l2_camera REJECTS a calibration file whose camera_name differs from this
        # parameter ("does not match ... in file"), and then publishes empty intrinsics
        # with no error beyond a warning. The yaml is written with the same name.
        DeclareLaunchArgument("camera_name", default_value="front_camera"),
        DeclareLaunchArgument(
            "pixel_format",
            default_value="YUYV",
            description="MJPG is faster but unsupported by v4l2_camera 0.7.3.",
        ),

        DeclareLaunchArgument(
            "camera_info_url",
            default_value="",
            description="file:///...yaml from camera_calibration. Empty means uncalibrated.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),

        LogInfo(msg="front camera: mono UVC (MJPG). Stereo calibration is NOT needed for v1.0; "
                    "single-camera intrinsics are, or AprilTag/rtabmap cannot give metric poses."),

        Node(
            package="usb_cam",
            executable="usb_cam_node_exe",
            name="camera",
            namespace="camera",
            output="screen",
            parameters=[{
                "video_device": video_device,
                "pixel_format": "mjpeg2rgb",
                "image_width": 1280,
                "image_height": 720,
                "framerate": 30.0,
                "camera_name": camera_name,
                "frame_id": "camera_optical_frame",
                "camera_info_url": camera_info_url,
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(EqualsSubstitution(driver, "usb_cam")),
        ),
        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="camera",
            namespace="camera",
            output="screen",
            parameters=[{
                "video_device": video_device,
                "pixel_format": pixel_format,
                # 640x480 is the largest YUYV mode that still runs 30 fps on this camera.
                "image_size": [640, 480],
                "time_per_frame": [1, 30],
                "camera_info_url": camera_info_url,
                "camera_name": camera_name,
                "output_encoding": "rgb8",
                "use_sim_time": use_sim_time,
            }],
            condition=IfCondition(EqualsSubstitution(driver, "v4l2")),
        ),
    ])
