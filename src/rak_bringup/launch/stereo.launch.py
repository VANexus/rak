"""Launch stereo capture with the existing driver defaults and topic names."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("left_camera_info_url", default_value=""),
        DeclareLaunchArgument("right_camera_info_url", default_value=""),
        Node(
            package="rak_stereo_driver",
            executable="stereo_node",
            output="screen",
            parameters=[{
                "device": LaunchConfiguration("device"),
                "left_camera_info_url": LaunchConfiguration("left_camera_info_url"),
                "right_camera_info_url": LaunchConfiguration("right_camera_info_url"),
            }],
        ),
    ])
