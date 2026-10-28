"""[AI] 底盘 + 臂 ros2_control 硬件链路 bringup（CAN v4 / SocketCAN 直连 F407，ADR-0016）。

启动 robot_state_publisher + controller_manager(rak_chassis) 并 spawn
joint_state_broadcaster、mecanum_drive_controller、arm_controller(JTC) 与
gripper_controller。上层工具（chassis_tui / arm_tui 等）只经 ROS 接口对话，
不直连 CAN（ADR-0016 决策 2）。

臂运动写者二选一（arm_motion_writer，ADR-0011 D1：底盘/臂各有多个候选写者，
但同一批 position 命令接口同一时刻只能有一个 active 控制器）：
  - jtc（默认）：arm_controller = JointTrajectoryController —— MoveIt / arm_tui 的
    FollowJointTrajectory 入口（轨迹语义，与 MoveIt 生态一致）。
  - wbc：arm_position_controller = ForwardCommandController，话题重映射到
    /wbc/arm_setpoints —— rak_wbc(HQP) 的位置流入口（速度级 WBC，ADR-0011）。
  - none：只起底盘，臂接口不激活（纯导航联调 / 臂未接线时）。

安全:
- allow_motion 默认 false —— 硬件永远只发 DISARMED 零目标，臂指令在 Session 层被抑制。
- allow_motion:=true 是显式运动许可，前提: 车轮架空 + 电池可控 + 有急停手段，
  臂周清空（臂也会解禁）。
- 麦轮 IK 在 mecanum_drive_controller(上位机), 轮级等比饱和在 F407。

用法:
  ros2 launch rak_bringup chassis_bringup.launch.py
  ros2 launch rak_bringup chassis_bringup.launch.py allow_motion:=true
  ros2 launch rak_bringup chassis_bringup.launch.py arm_motion_writer:=wbc   # WBC 接管臂
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    host_build_id = LaunchConfiguration("host_build_id").perform(context)
    allow_motion = LaunchConfiguration("allow_motion").perform(context)
    can_channel = LaunchConfiguration("can_channel").perform(context)
    arm_writer = LaunchConfiguration("arm_motion_writer").perform(context)
    # [AI] 值含空格，必须整体加引号：Command 走 shlex.split，不加会被拆成 4 个参数
    wheel_sign = LaunchConfiguration("wheel_sign").perform(context)

    urdf_path = LaunchConfiguration("urdf").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)

    robot_description = ParameterValue(
        # [AI] Command 真正执行 xacro，参数直通 URDF 中的 $(arg ...) 占位
        Command(
            f"xacro {urdf_path} host_build_id:={host_build_id} "
            f"allow_motion:={allow_motion} can_channel:={can_channel} "
            f'wheel_sign:="{wheel_sign}"'
        ),
        value_type=str,
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[
                controllers_file,
                {"robot_description": robot_description},
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_file],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mecanum_drive_controller",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_file],
            output="screen",
        ),
    ]

    # [AI] ADR-0012 待闭合项 #1 / ADR-0011 D1：臂运动写者二选一。只占 position
    # 接口，allow_motion=false 时命令在 Session 层被抑制（不会动）。
    if arm_writer == "jtc":
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller",
                           "--controller-manager", "/controller_manager",
                           "--param-file", controllers_file],
                output="screen",
            )
        )
    elif arm_writer == "wbc":
        # [AI] ForwardCommandController 的入话题固定是 ~/commands，重映射到
        # /wbc/arm_setpoints（rak_wbc 的臂位置流出口）。--controller-ros-args 传
        # 一条完整的 ros-args（含 -r）。
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_position_controller",
                           "--controller-manager", "/controller_manager",
                           "--param-file", controllers_file,
                           "--controller-ros-args", "-r ~/commands:=/wbc/arm_setpoints"],
                output="screen",
            )
        )
    elif arm_writer != "none":
        raise RuntimeError(
            f"arm_motion_writer must be one of jtc|wbc|none, got {arm_writer!r}")

    nodes.append(
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_file],
            output="screen",
        )
    )
    return nodes


def generate_launch_description():
    chassis_share = get_package_share_directory("rak_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("host_build_id", default_value="0xA0DE0002"),
        DeclareLaunchArgument(
            "allow_motion", default_value="false",
            description="运动许可开关；false=DISARMED(只握手+遥测)"),
        # [AI] ADR-0016: 链路唯一实现是 CAN v4（SocketCAN），无 transport/device 参数
        DeclareLaunchArgument(
            "can_channel", default_value="can0",
            description="SocketCAN 接口名"),
        # [AI] 臂运动写者：jtc(JTC，MoveIt/arm_tui) | wbc(HQP 位置流) | none
        DeclareLaunchArgument(
            "arm_motion_writer", default_value="jtc",
            description="臂控制器二选一：jtc=JointTrajectoryController(默认) / "
                        "wbc=arm_position_controller 收 /wbc/arm_setpoints / none"),
        # [AI] 每轮极性（装配事实）：2026-10-14 实车标定左侧 FL/RL 反接，
        # 默认 -1 1 -1 1。命令与反馈按此成对取反，回环自洽。
        DeclareLaunchArgument(
            "wheel_sign", default_value="-1 1 -1 1",
            description="四轮极性，空格分隔，顺序 FL FR RL RR；-1=该轮取反"),
        DeclareLaunchArgument(
            "urdf",
            default_value=f"{get_package_share_directory('rak_description')}/urdf/rak_chassis.urdf.xacro",
            description="底盘 URDF(xacro)路径"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=f"{chassis_share}/config/chassis_controllers.yaml",
            description="控制器配置文件"),
        OpaqueFunction(function=_launch_setup),
    ])
