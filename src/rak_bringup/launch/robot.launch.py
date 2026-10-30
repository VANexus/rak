"""整车顶层组合层：一条命令按 stage 起整栈，并由 system_manager 按故障域编排（ADR-0020）。

为什么单独一层（而不是塞进 sensors.launch.py）：
    sensors.launch.py 的职责是“传感器流 + 一个 RViz”，底盘、导航、agent 属于
    不同故障域与不同操作节奏。分层保持 ADR-0013“唯一组装层 = rak_bringup”，
    但把“组合关系”从口头约定变成可执行文件。

stage（互斥，默认 sensors）与它启用的故障域：
    sensors —— body + 打开的传感器域。不起底盘、不起导航。只读感知。
    teleop  —— body + link（ros2_control + 四轮 + 臂控制器）。手臂架空、台架用。
    nav     —— teleop + 定位 + 导航。allow_motion 仍默认 false。
    full    —— nav + task（执行器/仲裁决策层）+ expression（TTS/放音）。

    `domains` 由 stage **和**传感器开关一起推导：`lidar:=false` 时 lidar 域不进期望集，
    否则 manager 会诚实但碍事地报 "节点没出现在发现图里"。

supervise=false 是逃生门：回到 ADR-0020 之前的行为（各 launch 自管自启，没有编排者），
用来在排障时区分"栈坏了"与"编排坏了"。

用法：
    ros2 launch rak_bringup robot.launch.py
    ros2 launch rak_bringup robot.launch.py stage:=teleop allow_motion:=true
    ros2 launch rak_bringup robot.launch.py stage:=nav rviz:=false
    ros2 launch rak_bringup robot.launch.py stage:=full
    ros2 launch rak_bringup robot.launch.py stage:=sensors supervise:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

VALID_STAGES = ("sensors", "teleop", "nav", "full")

#: stage → 除传感器开关外固定启用的故障域（域名来自 system_manifest.yaml）。
STAGE_DOMAINS = {
    "sensors": ("body",),
    "teleop": ("body", "link"),
    "nav": ("body", "link", "localization", "navigation"),
    "full": ("body", "link", "localization", "navigation", "task", "expression"),
}
#: 传感器布尔开关与域名的对应（`camera_on`/`mic` 决定域是否进期望集）。
SENSOR_DOMAINS = (("lidar", "lidar"), ("camera_on", "camera"), ("imu", "imu"), ("mic", "mic"))


def _include(name, arguments, package="rak_bringup"):
    """按名字 include 同包 launch；arguments 里放 LaunchConfiguration/substitution。

    `package` 默认本包；编排入口在 rak_orchestration（ADR-0026 拆分），
    顶层只 include 它一次，避免出现第二个生命周期权威源。
    """
    share = get_package_share_directory(package)
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context, *args, **kwargs):
    def config(name):
        return LaunchConfiguration(name).perform(context)

    stage = config("stage")
    if stage not in VALID_STAGES:
        raise RuntimeError(f"stage must be one of {VALID_STAGES}, got {stage!r}")

    allow_motion = LaunchConfiguration("allow_motion")
    arm_writer = LaunchConfiguration("arm_motion_writer")
    can_channel = LaunchConfiguration("can_channel")
    camera = LaunchConfiguration("camera")
    rviz = LaunchConfiguration("rviz")
    motion_backend = LaunchConfiguration("motion_backend")
    supervise = config("supervise")
    transport = config("transport")

    chassis_args = {
        "allow_motion": allow_motion,
        "arm_motion_writer": arm_writer,
        "can_channel": can_channel,
    }
    # nav/full 下 chassis_bringup 的 robot_state_publisher 已发 TF，
    # 传感器层不能再发一份（ADR-0015：一个 frame 一个发布者）。
    sensors_owned_tf = stage == "sensors"
    actions = []

    if stage in ("teleop", "nav", "full"):
        # 传输二选一（ADR-0024）：两个主站**不得同时起**，同一总线双主站会把
        # 从站踢出 Operational —— 台架跑 tools/canopen_master.py 时不要同时 launch 本文件。
        if transport == "canopen":
            actions.append(_include("canopen_bringup.launch.py", {
                "allow_motion": allow_motion,
            }))
        else:
            actions.append(_include("chassis_bringup.launch.py", chassis_args))

    if stage in ("sensors", "nav", "full"):
        actions.append(_include("sensors.launch.py", {
            "publish_robot_tf": "true" if sensors_owned_tf else "false",
            # 轮式遥测由 chassis_bringup 提供，sensors 不重复起底盘
            "chassis": "false",
            "camera": camera,
            "rviz": rviz,
            "lidar": LaunchConfiguration("lidar"),
            "camera_on": LaunchConfiguration("camera_on"),
            "imu": LaunchConfiguration("imu"),
            "mic": LaunchConfiguration("mic"),
            # slam/nav2 由本文件负责，sensors 里那份 slam include 关掉避免双起
            "slam": "false",
            "supervised": supervise,
        }))

    if stage in ("nav", "full"):
        # 只 include nav2.launch.py —— 它内部已经 include slam.launch.py 了。
        # 之前这里同时显式 include slam，等于两份 slam_toolbox + 两个 rf2o
        # + 两个 odom→base_footprint 发布者（违反 ADR-0015）。
        actions.append(_include("nav2.launch.py", {
            "motion_backend": motion_backend,
            "publish_robot_tf": "false",
            "odom_source": LaunchConfiguration("odom_source"),
        }))

    if stage == "full":
        # supervised 必须一路传下去：漏传就会"节点自启 + manager 也想驱动"，
        # 出现两个生命周期权威源（ADR-0020 §3）。今天只有 task 传了，其余三个是漏的。
        actions.append(_include("agent.launch.py", {"supervised": supervise}))
        actions.append(_include("speech.launch.py", {"supervised": supervise}))
        actions.append(_include("task.launch.py", {"supervised": supervise}))

    if stage == "teleop":
        # 台架只需要 TF + 关节可视，不拉传感器
        actions.append(_include("sensors.launch.py", {
            "publish_robot_tf": "false",
            "lidar": "false",
            "camera_on": "false",
            "imu": "false",
            "mic": "false",
            "chassis": "false",
            "rviz": rviz,
            "supervised": supervise,
        }))

    if supervise == "true":
        domains = list(STAGE_DOMAINS[stage])
        domains += [
            domain for arg, domain in SENSOR_DOMAINS
            if stage != "teleop" and config(arg) == "true"
        ]
        actions.append(_include("supervisor.launch.py", {
            "domains": ",".join(sorted(set(domains))),
        }, package="rak_orchestration"))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stage", default_value="sensors",
            description="sensors | teleop | nav | full"),
        DeclareLaunchArgument(
            "allow_motion", default_value="false",
            description="运动许可；false=硬件只握手+遥测，不驱动"),
        DeclareLaunchArgument(
            "arm_motion_writer", default_value="jtc",
            description="臂控制器二选一：jtc / wbc / none"),
        DeclareLaunchArgument("can_channel", default_value="can0"),
        # 传输（ADR-0024 + 2026-10-30 修订）：下位机 v4 已整体下线（R75/A16，
        # 无回滚分支），CanSession 对真板**无效**，只剩软件夹具用途。
        # 台架/联调的实际主链路 = 伞形仓 tools/canopen_master.py（Orin 上运行）。
        DeclareLaunchArgument(
            "transport", default_value="canopen", choices=["can_v4", "canopen"],
            description="底盘控制链：canopen（真机唯一）| can_v4（仅软件夹具；真板无效）"),
        DeclareLaunchArgument(
            "camera", default_value="gemini335l",
            description="深度相机后端：gemini335l | astra（透传给 sensors）"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "motion_backend", default_value="sim",
            description="nav2 运动后端；真机联调时不传（走 nav2 默认）"),
        DeclareLaunchArgument("odom_source", default_value="rf2o",
                              choices=["rf2o", "ekf", "none"]),
        # ---- 传感器开关：同时决定该传感器域是否进 manager 的期望集 ----
        DeclareLaunchArgument("lidar", default_value="true"),
        DeclareLaunchArgument("camera_on", default_value="true"),
        DeclareLaunchArgument("imu", default_value="true"),
        DeclareLaunchArgument("mic", default_value="true"),
        DeclareLaunchArgument(
            "supervise", default_value="true", choices=["true", "false"],
            description="false = 不起 system_manager（ADR-0020 之前的行为，排障用）"),
        OpaqueFunction(function=_launch_setup),
    ])
