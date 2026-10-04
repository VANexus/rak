"""Launch Astra image streams; installation TF must come from measured calibration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("depth_frame_id", default_value="camera_depth_frame"),
        Node(
            package="rak_astra_driver",
            executable="astra_node",
            output="screen",
            parameters=[{
                "color_frame_id": LaunchConfiguration("color_frame_id"),
                "depth_frame_id": LaunchConfiguration("depth_frame_id"),
            }],
        ),
    ])
