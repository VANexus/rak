"""Draft RGB-D mapping entry for later deployment — do not run alongside slam_toolbox.

This file is prepared outside the robot workspace. To deploy:
  1. copy this file to `src/rak_bringup/launch/rtabmap_mapping.launch.py`;
  2. copy `rtabmap_rgbd_mapping.yaml` to `src/rak_bringup/config/`;
  3. rebuild `rak_bringup` and source the workspace.

Ownership (ADR-0002 / ADR-0015):
  map -> odom        : rtabmap (this launch)
  odom -> base_footprint : robot_localization EKF (separate launch)
  camera optical frames  : the Orbbec driver
  body tree          : robot_state_publisher (separate launch)

Inputs: /camera/color/image_raw, /camera/depth/image_raw,
        /camera/color/camera_info, /odometry/filtered
Outputs: map -> odom, /map, RTAB-Map database.

This launch does NOT start the camera, the EKF, or Nav2. It also does not start
rtabmap's own visual odometry (`visual_odometry:=false`), because /odometry/filtered
is the pose prior and the EKF owns odom -> base_footprint.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory("rak_bringup")
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"), "launch", "rtabmap.launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "database_path",
                default_value=os.path.expanduser("~/.ros/rtabmap_rgbd.db"),
                description="建图数据库；只在这里首次生成，之后复用。",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/odometry/filtered",
                description="EKF 输出；没有 EKF 时传 /odom（rf2o）。",
            ),
            DeclareLaunchArgument(
                "cfg",
                default_value=os.path.join(
                    bringup_share, "config", "rtabmap_rgbd_mapping.yaml"
                ),
                description="RTAB-Map 参数文件。",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rtabmap_launch),
                launch_arguments={
                    # mapping, not localization
                    "localization": "false",
                    "database_path": LaunchConfiguration("database_path"),
                    "frame_id": "base_footprint",
                    "odom_topic": LaunchConfiguration("odom_topic"),
                    # RGB-D input; the upstream launch argument is `depth` (not `subscribe_depth`)
                    "depth": "true",
                    "subscribe_rgb": "true",
                    "subscribe_rgbd": "false",
                    "rgb_topic": "/camera/color/image_raw",
                    "depth_topic": "/camera/depth/image_raw",
                    "camera_info_topic": "/camera/color/camera_info",
                    "approx_sync": "true",
                    "queue_size": "10",
                    "sync_queue_size": "10",
                    # EKF owns odom -> base_footprint; do not run a second visual odometry
                    "visual_odometry": "false",
                    "cfg": LaunchConfiguration("cfg"),
                    "rtabmap_viz": "false",
                    "rviz": "false",
                    "use_sim_time": LaunchConfiguration("use_sim_time"),
                }.items(),
            ),
        ]
    )
