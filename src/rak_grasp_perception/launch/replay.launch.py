from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    names = {"snapshot": "", "result": "", "roi_output": "roi.json"}
    return LaunchDescription([
        *[DeclareLaunchArgument(k, default_value=v) for k, v in names.items()],
        Node(package="rak_grasp_perception", executable="replay_grasps",
             parameters=[{k: LaunchConfiguration(k) for k in names}], output="screen"),
    ])
