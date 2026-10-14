"""[AI] 底盘麦轮键盘测试节点入口。

独立启动 chassis_key_test(键盘 WASD/QE -> Twist 发布), 不包含 Nav2 /
控制器 / 串口。目标话题由 topic 参数指定:
  - 默认 /mecanum_drive_controller/reference (TwistStamped): 直连
    Jazzy mecanum_drive_controller 的 ~/reference(唯一订阅入口);
    需外层已有 controller_manager + rak_chassis 在跑;
  - topic:=/cmd_vel         : 走 Nav2 完整链路(平滑/碰撞监视);
  - topic:=/cmd_vel_sim     : 仿真桩。

⚠️ 键盘在 launch 下不可用: ros2 launch 给子进程的 stdin 是 PIPE 而非 tty,
   termios.tcgetattr(0) 失败 → 只按零目标持续发布(打印"键盘输入禁用")。
   需要键盘请直接 `ros2 run rak_commissioning chassis_key_test`。
   本 launch 仅用于无键盘的持续零目标/话题连通性验证。

⚠️ 仅为测试输入源; 电机安全 ARM 由 rak_chassis 底层把关, 默认 DISARMED。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    topic = LaunchConfiguration("topic")
    stamped = LaunchConfiguration("stamped")
    return LaunchDescription([
        DeclareLaunchArgument(
            "topic", default_value="/mecanum_drive_controller/reference",
            description="速度目标话题"),
        DeclareLaunchArgument(
            "stamped", default_value="true",
            description="发布 TwistStamped(true) 还是 Twist(false)"),
        Node(
            package="rak_commissioning",
            executable="chassis_key_test",
            name="chassis_key_test",
            output="screen",
            parameters=[{
                "topic": topic,
                # [AI] LaunchConfiguration 求值是字符串, 必须显式标注 bool,
                # 否则与 declare_parameter("stamped", True) 的类型不匹配。
                "stamped": ParameterValue(stamped, value_type=bool),
                "rate": 20.0,
            }],
        ),
    ])
