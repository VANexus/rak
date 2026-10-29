"""生命周期编排入口（ADR-0020）：起 `rak_lifecycle/system_manager`，按 manifest 驱动全栈。

    ros2 launch rak_bringup supervisor.launch.py                       # 真机清单，全部域
    ros2 launch rak_bringup supervisor.launch.py domains:=task,link    # 只管这两个域
    ros2 launch rak_bringup supervisor.launch.py profile:=fixture      # 软件夹具，不碰硬件

为什么需要一个单独的入口，而不是把转移写在各个 launch 里：
    `slam.launch.py` 里那套 `EmitEvent(ChangeState(...))` + `OnStateTransition` 只能在
    **进程启动那一刻**编排两个节点；launch 一退出，没人还能重启 slam_toolbox，也没人知道
    整机现在能不能用。跨域的门控、故障降级与"原地恢复"需要一个长期在场的编排者。

组合规矩（避免造出第二个生命周期权威源）：
    - 顶层 `robot.launch.py` 只 include 本文件**一次**；
    - 被 `system_manager` 拥有的节点在各自 launch 里必须是 `LifecycleNode` 且不自启
      （`supervised:=true` 传入时）；
    - 单独跑某个 launch（`supervised:=false`，默认）时它自己 autostart —— 宁可用不上编排，
      也不要留一个"进程在但 unconfigured"的静默节点，那是本仓库记录过的最坑故障形态。

`profile:=fixture` 用 `system_manifest_fixture.yaml` + 软件夹具（fake navigation /
fake joint controllers）：域结构、所有权与判据形状和真机一致，用来在没有硬件的环境里
端到端验证编排本身。fake telemetry 是命令运动估算，不是实测（`docs/hardware_acceptance.md`）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

PROFILES = {
    "real": "system_manifest.yaml",
    "fixture": "system_manifest_fixture.yaml",
}


def _fixture_components(context):
    """夹具档：组件由本文件自己起，因此不依赖任何硬件或传感器。"""
    share = get_package_share_directory("rak_task_executor")
    waypoints = os.path.join(share, "config", "waypoints_sim.yaml")
    return [
        # 夹具里唯一被 manager 拥有的组件：与真机清单同一份实现（TaskExecutor），
        # 所以这条路径验证的是**真代码**，不是替身。
        LifecycleNode(
            package="rak_task_executor",
            executable="task_executor",
            name="task_executor",
            namespace="",
            output="screen",
            parameters=[{"waypoints_file": waypoints, "slow_planner": "local"}],
        ),
        # 两个 observed 夹具：仍是普通进程，负责让上游域能被"看到在位"，
        # 也用来演示"上游停发 → 下游降级"。
        Node(
            package="rak_task_executor",
            executable="fake_navigation",
            output="screen",
            parameters=[{
                "goal_duration_sec": ParameterValue(
                    LaunchConfiguration("goal_duration_sec"), value_type=float
                ),
            }],
        ),
    ]


def _launch_setup(context, *args, **kwargs):
    profile = LaunchConfiguration("profile").perform(context)
    if profile not in PROFILES:
        raise RuntimeError(f"profile must be one of {sorted(PROFILES)}, got {profile!r}")

    manifest = LaunchConfiguration("manifest").perform(context)
    if not manifest:
        manifest = os.path.join(
            get_package_share_directory("rak_bringup"), "config", PROFILES[profile]
        )

    actions = [
        Node(
            package="rak_lifecycle",
            executable="system_manager",
            name="rak_system_manager",
            output="screen",
            parameters=[{
                "manifest": manifest,
                # 值是 substitution，布尔参数必须显式定型，否则 rclpy 收到字符串会 WRONG_TYPE
                "autostart": ParameterValue(
                    LaunchConfiguration("autostart"), value_type=bool
                ),
                "enabled_domains": LaunchConfiguration("domains"),
            }],
        )
    ]
    if profile == "fixture":
        actions.extend(_fixture_components(context))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "profile", default_value="real", choices=list(PROFILES),
            description="real = 真机清单；fixture = 软件夹具（不碰硬件）",
        ),
        DeclareLaunchArgument(
            "manifest", default_value="",
            description="覆盖清单路径；留空按 profile 取 rak_bringup/config 里的那份",
        ),
        DeclareLaunchArgument(
            "domains", default_value="",
            description="逗号分隔的启用域；留空 = 清单里全部启用的域",
        ),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="false = manager 起来但不拉起任何域（由操作员用 rak_system start 逐域拉起）",
        ),
        DeclareLaunchArgument(
            "goal_duration_sec", default_value="60.0",
            description="仅 profile:=fixture 用 —— 假导航目标的持有时长 [s]",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
