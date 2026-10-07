"""Lidar driver only. Useful on its own for de-risking the serial link.

    ros2 launch rak_sensor_bringup lidar.launch.py [serial_port:=/dev/ttyUSB0]

The packaged config points at `/dev/wheeltec_lidar`, the udev alias from
`udev/99-wheeltec-lidar.rules`. That rule needs root to install, so when the alias
is absent this launch falls back to the device's stable `/dev/serial/by-id` name
rather than failing with an unhelpful `open_port ERROR`.
"""

import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: The N10_P hangs off a Silicon Labs CP2102 (10c4:ea60). Matching the bridge rather
#: than the tty number is what makes the fallback stable across replugging.
_BY_ID_GLOBS = (
    "/dev/serial/by-id/*CP210*",
    "/dev/serial/by-id/*Silicon_Labs*",
)


def _resolve_port(context) -> str:
    requested = LaunchConfiguration("serial_port").perform(context)
    if os.path.exists(requested):
        return requested
    for pattern in _BY_ID_GLOBS:
        for candidate in sorted(glob.glob(pattern)):
            print(
                f"[lidar.launch] {requested} is missing (udev alias not installed); "
                f"using {candidate}",
                flush=True,
            )
            return candidate
    # Nothing matched: hand the requested path back so the driver's own error names it.
    return requested


def _lidar_node(context):
    share = get_package_share_directory("rak_sensor_bringup")
    port = _resolve_port(context)
    return [
        Node(
            package="lslidar_driver",
            executable="lslidar_driver_node",
            name="lslidar_driver_node",
            output="screen",
            parameters=[
                os.path.join(share, "config", "lslidar_n10.yaml"),
                {"serial_port_": ParameterValue(port, value_type=str)},
            ],
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


def generate_launch_description():
    return LaunchDescription(
        [
            # The packaged config points at the /dev/wheeltec_lidar udev alias. That rule
            # is not installed on every image, so allow overriding the device instead of
            # editing the pinned config.
            DeclareLaunchArgument(
                "serial_port",
                default_value="/dev/wheeltec_lidar",
                description="Serial device for the N10_P.",
            ),
            OpaqueFunction(function=_lidar_node),
        ]
    )
