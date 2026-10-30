"""指标聚合节点（ADR-0020 的观测半边）。

    ros2 launch rak_metrics rak_metrics.launch.py
    ros2 launch rak_metrics rak_metrics.launch.py http_port:=8090
    curl -s http://127.0.0.1:8088/metrics | python3 -m json.tool

`supervised:=true` 时由 `system_manager` 驱动生命周期，本文件不自启。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from rak_orchestration.launch_helpers import managed


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 由 system_manager 驱动生命周期（本文件不自启）"),
        DeclareLaunchArgument("http_port", default_value="8088"),
        DeclareLaunchArgument("http_host", default_value="127.0.0.1"),
        DeclareLaunchArgument(
            "probes", default_value="",
            description="YAML 列表，覆盖默认探针（话题/类型/标称频率/依据）"),
        *managed(
            package="rak_metrics",
            executable="rak_metrics",
            name="rak_metrics",
            output="screen",
            parameters=[{
                "http_port": ParameterValue(LaunchConfiguration("http_port"), value_type=int),
                "http_host": LaunchConfiguration("http_host"),
                "probes": LaunchConfiguration("probes"),
            }],
        ),
    ])
