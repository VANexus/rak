"""Launch the tool executor.

Motion-class tools (arm.home / task.goto) stay rejected until BOTH gates
are open: `allow_motion:=true` on this launch and `operator_confirmed` on
each proposal. Bench default is false.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("allow_motion", default_value="false"),
        DeclareLaunchArgument("audit_dir", default_value="data/episodes"),
    ]
    node = Node(
        package="rak_tool_executor",
        executable="tool_executor",
        name="tool_executor",
        output="screen",
        parameters=[
            {
                "allow_motion": LaunchConfiguration("allow_motion"),
                "audit_dir": LaunchConfiguration("audit_dir"),
            }
        ],
    )
    return LaunchDescription(args + [node])
