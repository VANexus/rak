"""CANopen v5 底盘/机械臂控制链（ADR-0024）—— 与 `chassis_bringup.launch.py` **互斥**。

为什么单独一个 launch 而不是给 chassis_bringup 加分支：
    两者的硬件插件、参数文件、失败模式都不同（v4 是 `CanSession`，v5 是 ros2_control 的
    CANopen 插件 + EDS）。塞进同一个文件会让"当前到底跑哪条链"变成读代码才能回答的问题。
    `robot.launch.py` 用 `transport:=canopen|can_v4` 二选一，同一总线**不得同时起两个主站**。

⚠️ 状态：**未编译、未实机验证**（桌面机没有 ros2_canopen）。
    上机第一步：`ros2 control list_hardware_interfaces`，把实际接口名回填
    `rak_canopen/config/canopen_controllers.yaml`，差异写进 ADR-0024 待闭合项 #2。

用法：
    ros2 launch rak_bringup canopen_bringup.launch.py
    ros2 launch rak_bringup canopen_bringup.launch.py allow_motion:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description = get_package_share_directory("rak_description")
    canopen = get_package_share_directory("rak_canopen")

    robot_description = ParameterValue(
        Command(["xacro ", os.path.join(description, "urdf", "rak.urdf.xacro")]),
        value_type=str,
    )

    # 硬件插件名可覆盖：ros2_canopen 各版本对 RobotSystem 的导出名不完全一致，
    # 写死会让"装了但名字不同"表现成插件加载失败（一个很难查的错）。
    hardware_plugin = LaunchConfiguration("hardware_plugin")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            os.path.join(canopen, "config", "canopen_controllers.yaml"),
            {"robot_description": robot_description},
            # 插件名同时作为参数下发：既让 launch 参数真的被消费，也让
            # `ros2 param get` 能回答"这次到底想加载哪个插件"。
            {"canopen_hardware": {"rosplugin": hardware_plugin}},
        ],
        # 插件通过 ros2_control 的 <ros2_control> 标签加载；这里的参数只是让
        # 参数文件里的 canopen_hardware 段落有宿主。
        remappings=[("/controller_manager/robot_description", "/robot_description")],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    drive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("allow_motion", default_value="false",
                              description="运动许可；false 时只握手+遥测"),
        DeclareLaunchArgument("hardware_plugin", default_value="canopen_ros2_control/RobotSystem",
                              description="ros2_control 硬件插件导出名（按实际安装核对）"),
        controller_manager,
        # spawner 必须等 controller_manager 起来；用 OnProcessExit 串行化，
        # 避免三个 spawner 同时打一个还没就绪的服务。
        joint_state_broadcaster,
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster, on_exit=[drive])),
        RegisterEventHandler(OnProcessExit(target_action=drive, on_exit=[arm])),
    ])
