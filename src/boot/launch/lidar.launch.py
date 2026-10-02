"""Lidar driver only. Useful on its own for de-risking the serial link.

    ros2 launch boot lidar.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("boot")
    return LaunchDescription(
        [
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                name="lslidar_driver_node",
                output="screen",
                parameters=[os.path.join(share, "config", "lslidar_n10.yaml")],
            ),
            # The driver's N10_P branch emits 2*count_num readings under
            # single-layer metadata, which karto rejects outright. Normalise
            # /scan_raw -> /scan here so every consumer (rf2o, slam_toolbox,
            # anything added later) only ever sees a well-formed scan.
            Node(
                package="laser_filters",
                executable="scan_to_scan_filter_chain",
                name="scan_to_scan_filter_chain",
                output="screen",
                parameters=[os.path.join(share, "config", "lidar_scan_filter.yaml")],
                remappings=[("scan", "/scan_raw"), ("scan_filtered", "/scan")],
            ),
        ]
    )
