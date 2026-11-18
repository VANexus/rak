"""指标聚合节点（ADR-0020 的观测半边）。

    ros2 launch rak_metrics rak_metrics.launch.py
    ros2 launch rak_metrics rak_metrics.launch.py http_port:=8090
    curl -s http://127.0.0.1:8088/metrics | python3 -m json.tool

`supervised:=true` 时由 `system_manager` 驱动生命周期，本文件不自启。

slip_watch（抗打滑 L1 检测，firmware-capability-spec §7.2）：默认随本文件
启动，只观测（残差 p50/p95 进 /rak/metrics），sigma_baseline 未标定前
不产生打滑判定。**导航栈 bringup（nav2.launch.py）里也起一份 slip_watch**
（接 L2 中继 odom_cov_guard 与可选 L3 gate）——同名节点只能跑一份，
随 robot.launch.py 整栈跑时不要另起本文件（或与 nav2 二选一）。
L3 的 /cmd_vel 限幅通路默认关闭（gate_enabled:=false）。
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
        # [AI] 抗打滑阈值（§7.2）：sigma_baseline=0 表示未标定——由首日基线
        # （本节点发布的 residual_p50/p95_mrad_s，或 slip_sigma 基线工具）回填后再启用判定。
        DeclareLaunchArgument("slip_sigma_baseline", default_value="0.0"),
        DeclareLaunchArgument(
            "slip_gate", default_value="false",
            description="L3 gate_enabled 透传（实际换线由 nav2.launch.py 的 slip_gate 负责）"),
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
        *managed(
            package="rak_metrics",
            executable="slip_watch",
            name="slip_watch",
            output="screen",
            parameters=[{
                "sigma_baseline": ParameterValue(
                    LaunchConfiguration("slip_sigma_baseline"), value_type=float),
                "gate_enabled": ParameterValue(
                    LaunchConfiguration("slip_gate"), value_type=bool),
            }],
        ),
    ])
