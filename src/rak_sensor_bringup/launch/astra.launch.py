"""Launch the tested Astra driver; calibration and hardware TF still need validation."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="rak_astra_driver",
            executable="astra_node",
            output="screen",
        ),
    ])
