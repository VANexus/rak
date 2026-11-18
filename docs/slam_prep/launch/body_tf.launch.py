"""Single body-tree publisher for the lightweight local-costmap stack.

Use only when no chassis_bringup robot_state_publisher is running. It loads
the same whole-robot xacro as slam.launch.py, so the sensor mounts stay in the
URDF rather than ad-hoc static transforms.

    ros2 launch slam_prep/launch/body_tf.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory("rak_description"),
        "urdf",
        "rak_chassis.urdf.xacro",
    )
    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", urdf]), value_type=str
                        )
                    }
                ],
            )
        ]
    )
