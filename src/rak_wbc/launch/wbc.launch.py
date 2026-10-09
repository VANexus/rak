"""[AI] rak_wbc 启动入口（ADR-0011）。

只启动仲裁节点本身；底盘/臂的 ros2_control 由 rak_bringup（chassis_bringup.launch）或整机链路启动
负责。默认 enabled=false —— 启动后需显式调用 /wbc/enable 且现场安全确认后才发运动。

用法:
  ros2 launch rak_wbc wbc.launch.py
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    parameters = get_package_share_directory("rak_wbc") + "/config/wbc_params.yaml"
    return LaunchDescription(
        [
            Node(
                package="rak_wbc",
                executable="wbc_node",
                name="wbc",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )