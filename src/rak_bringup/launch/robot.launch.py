"""整车顶层组合层：一条命令按 stage 起整栈，避免操作员手工拼 7 个 launch。

为什么单独一层（而不是塞进 sensors.launch.py）：
    sensors.launch.py 的职责是“传感器流 + 一个 RViz”，底盘、导航、agent 属于
    不同故障域与不同操作节奏。分层保持 ADR-0013“唯一组装层 = rak_bringup”，
    但把“组合关系”从口头约定变成可执行文件。

stage（互斥，默认 sensors）：
    sensors —— 雷达 / 相机 / IMU / 语音，不起底盘、不起导航。只读感知。
    teleop  —— chassis_bringup（ros2_control + 四轮 + 臂控制器）。手臂架空、台架用。
    nav     —— chassis_bringup + sensors + slam + nav2。allow_motion 仍默认 false。
    full    —— nav + agent（决策层）+ speech（离线 ASR/TTS）。

参数语义：
    allow_motion        透传底盘/臂的运动许可（默认 false，永不自动驱动）。
    arm_motion_writer   透传 jtc|wbc|none（见 chassis_bringup.launch.py）。
    can_channel         透传 SocketCAN 接口名。
    camera / rviz       nav 阶段传感器的选择与可视开关。
    motion_backend      透传 nav2 的 sim|stub（真机为不传，走默认）。

用法：
    ros2 launch rak_bringup robot.launch.py
    ros2 launch rak_bringup robot.launch.py stage:=teleop allow_motion:=true
    ros2 launch rak_bringup robot.launch.py stage:=nav rviz:=false
    ros2 launch rak_bringup robot.launch.py stage:=full
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

VALID_STAGES = ("sensors", "teleop", "nav", "full")


def _include(name, arguments):
    """按名字 include 同包 launch；arguments 里放 LaunchConfiguration/substitution。"""
    share = get_package_share_directory("rak_bringup")
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share, "launch", name)),
        launch_arguments=arguments.items(),
    )


def _launch_setup(context, *args, **kwargs):
    stage = LaunchConfiguration("stage").perform(context)
    if stage not in VALID_STAGES:
        raise RuntimeError(f"stage must be one of {VALID_STAGES}, got {stage!r}")

    allow_motion = LaunchConfiguration("allow_motion")
    arm_writer = LaunchConfiguration("arm_motion_writer")
    can_channel = LaunchConfiguration("can_channel")
    camera = LaunchConfiguration("camera")
    rviz = LaunchConfiguration("rviz")
    motion_backend = LaunchConfiguration("motion_backend")

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
        actions.append(_include("chassis_bringup.launch.py", chassis_args))

    if stage in ("sensors", "nav", "full"):
        actions.append(_include("sensors.launch.py", {
            "publish_robot_tf": "true" if sensors_owned_tf else "false",
            # 轮式遥测由 chassis_bringup 提供，sensors 不重复起底盘
            "chassis": "false",
            "camera": camera,
            "rviz": rviz,
        }))

    if stage in ("nav", "full"):
        # ADR-0015：body TF 只允许 chassis_bringup 的 robot_state_publisher 发一份
        actions.append(_include("slam.launch.py", {"publish_robot_tf": "false"}))
        actions.append(_include("nav2.launch.py", {
            "motion_backend": motion_backend,
            "publish_robot_tf": "false",
        }))

    if stage == "full":
        actions.append(_include("agent.launch.py", {}))
        actions.append(_include("speech.launch.py", {}))

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
        }))

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
        DeclareLaunchArgument(
            "camera", default_value="gemini335l",
            description="深度相机后端：gemini335l | astra（透传给 sensors）"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument(
            "motion_backend", default_value="sim",
            description="nav2 运动后端；真机联调时不传（走 nav2 默认）"),
        OpaqueFunction(function=_launch_setup),
    ])
