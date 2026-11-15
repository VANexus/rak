"""Visual odometry (VIO's vision half) from rtabmap_odom -> /odometry/vio.

    ros2 launch rak_bringup vio.launch.py                       # rgbd, depth camera
    ros2 launch rak_bringup vio.launch.py source:=stereo        # stereo pair (needs calib)
    ros2 launch rak_bringup vio.launch.py source:=none          # no VIO input

Why rtabmap_odom: it is already installed (apt 0.23.13) and is the same feature
engine the RTAB-Map SLAM/localization node uses, so mapping and localization
share one implementation. No new wheel is built here.

Ownership (ADR-0002 / ADR-0015): this node publishes ONLY `/odometry/vio`
(no TF). `odom -> base_footprint` stays with the robot_localization EKF
(ekf_4d.yaml, odom2 input), which also fuses wheel odometry, rf2o and the IMU.

Inputs:
  rgbd    /camera/color/image_raw + /camera/color/camera_info
          /camera/depth/image_raw + /camera/depth/camera_info
  stereo  /stereo/left/image_rect + /stereo/right/image_rect + .../camera_info
          (stereo_image_proc rectifies; needs a real stereo calibration file)

Honest gate: with no calibration file the stereo branch produces nothing usable,
so it is opt-in. `source:=rgbd` needs the Orbbec/Astra driver up (`sensors.launch.py`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _compose(context, cfg):
    source = LaunchConfiguration("source").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    if source == "none":
        return []
    common = [cfg, {"use_sim_time": use_sim_time}]
    if source == "rgbd":
        return [Node(
            package="rtabmap_odom", executable="rgbd_odometry", name="rgbd_odometry",
            output="screen", parameters=common,
            remappings=[
                ("rgb/image", "/camera/color/image_raw"),
                ("rgb/camera_info", "/camera/color/camera_info"),
                ("depth/image", "/camera/depth/image_raw"),
                ("depth/camera_info", "/camera/depth/camera_info"),
                ("odom", "/odometry/vio"),
            ],
            # rtabmap_odom 的默认输出话题是 `odom`；remap 里已经指向 /odometry/vio。
        )]
    return [Node(
        package="rtabmap_odom", executable="stereo_odometry", name="stereo_odometry",
        output="screen", parameters=common,
        remappings=[
            ("left/image_rect", "/stereo/left/image_rect"),
            ("left/camera_info", "/stereo/left/camera_info"),
            ("right/image_rect", "/stereo/right/image_rect"),
            ("right/camera_info", "/stereo/right/camera_info"),
            ("odom", "/odometry/vio"),
        ],
    )]


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("rak_bringup"), "config", "vio_rtabmap.yaml")
    return LaunchDescription([
        DeclareLaunchArgument(
            "source",
            default_value="rgbd",
            choices=["rgbd", "stereo", "none"],
            description="Which rtabmap_odom flavour feeds /odometry/vio.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_compose, kwargs={"cfg": cfg}),
    ])
