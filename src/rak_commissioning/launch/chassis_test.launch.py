"""[AI] 底盘麦轮键盘测试节点入口。

独立启动 chassis_key_test(键盘 WASD/QE -> Twist 发布), 不包含 Nav2 /
控制器 / 串口。目标话题由 topic 参数指定:
  - 默认 /cmd_vel_unstamped: 直连 mecanum_drive_controller(硬件链路),
    需外层已有 controller_manager + rak_hardware 在跑;
  - topic:=/cmd_vel         : 走 Nav2 完整链路(平滑/碰撞监视);
  - topic:=/cmd_vel_sim     : 仿真桩。

⚠️ 仅为测试输入源; 电机安全 ARM 由 rak_hardware 底层把关, 默认 DISARMED。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    topic = LaunchConfiguration("topic")
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic", default_value="/cmd_vel_unstamped",
            description="Twist 目标话题"),
        Node(
            package="rak_commissioning",
            executable="chassis_key_test",
            name="chassis_key_test",
            output="screen",
            parameters=[{
                "topic": topic,
                "stamped": False,
                "rate": 20.0,
            }],
        ),
    ])
