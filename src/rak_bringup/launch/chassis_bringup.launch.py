"""[AI] 底盘 ros2_control 硬件链路 bringup（USB v2 直连 F407）。

启动 robot_state_publisher + controller_manager(rak_chassis) 并 spawn
joint_state_broadcaster 与 mecanum_drive_controller。

安全:
- allow_motion 默认 false —— 硬件永远只发 DISARMED 零目标。
- allow_motion:=true 是显式运动许可，前提: 车轮架空 + 电池可控 + 有急停手段。
- 麦轮 IK 在 mecanum_drive_controller(上位机), 轮级等比饱和在 F407。

用法:
  ros2 launch rak_bringup chassis_bringup.launch.py
  ros2 launch rak_bringup chassis_bringup.launch.py allow_motion:=true
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _launch_setup(context, *args, **kwargs):
    device = LaunchConfiguration("device").perform(context)
    host_build_id = LaunchConfiguration("host_build_id").perform(context)
    allow_motion = LaunchConfiguration("allow_motion").perform(context)

    urdf_path = LaunchConfiguration("urdf").perform(context)
    controllers_file = LaunchConfiguration("controllers_file").perform(context)

    robot_description = ParameterValue(
        # [AI] Command 真正执行 xacro，参数直通 URDF 中的 $(arg ...) 占位
        Command(
            f"xacro {urdf_path} device:={device} host_build_id:={host_build_id} "
            f"allow_motion:={allow_motion}"
        ),
        value_type=str,
    )

    return [
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


def generate_launch_description():
    chassis_share = get_package_share_directory("rak_bringup")
    return LaunchDescription([
        DeclareLaunchArgument(
            "device",
            default_value="/dev/serial/by-id/usb-WCH.CN_USB_Single_Serial_0002-if00",
            description="F407 稳定 USB 设备路径(必须 /dev/serial/by-id/ 前缀)"),
        DeclareLaunchArgument("host_build_id", default_value="0xA0DE0002"),
        DeclareLaunchArgument(
            "allow_motion", default_value="false",
            description="运动许可开关；false=DISARMED(只握手+遥测)"),
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
