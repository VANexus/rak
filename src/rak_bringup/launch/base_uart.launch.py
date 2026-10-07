"""Standalone F407 host link. Physical access requires explicit launch and ARM gates."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/wheel_motor",
            description="Configured F407 base UART path. Verify device identity "
                        "before enabling hardware; the default hardware gate is false.",
        ),
        DeclareLaunchArgument("baud", default_value="921600"),
        DeclareLaunchArgument(
            "hardware_enable",
            default_value="false",
            description="Allow base_uart_driver to open the port and send "
                        "SET_BASE_VELOCITY to the F407; false = device is not "
                        "opened and no serial traffic is sent.",
        ),
        DeclareLaunchArgument("max_vx_m_s", default_value="0.0"),
        DeclareLaunchArgument("max_vy_m_s", default_value="0.0"),
        DeclareLaunchArgument("max_wz_rad_s", default_value="0.0"),
        Node(
            package="rak_base_driver",
            executable="base_uart_node",
            name="base_uart_driver",
            output="screen",
            parameters=[{
                "port": LaunchConfiguration("port"),
                "baud": LaunchConfiguration("baud"),
                "allow_hardware": ParameterValue(
                    LaunchConfiguration("hardware_enable"), value_type=bool
                ),
                "max_vx_m_s": ParameterValue(
                    LaunchConfiguration("max_vx_m_s"), value_type=float
                ),
                "max_vy_m_s": ParameterValue(
                    LaunchConfiguration("max_vy_m_s"), value_type=float
                ),
                "max_wz_rad_s": ParameterValue(
                    LaunchConfiguration("max_wz_rad_s"), value_type=float
                ),
                "cmd_vel_topic": "/cmd_vel",
                "telemetry_topic": "/base/telemetry",
            }],
        ),
    ])
