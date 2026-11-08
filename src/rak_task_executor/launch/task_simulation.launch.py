"""Start a deterministic software-only task lifecycle fixture.

    ros2 launch rak_task_executor task_simulation.launch.py
    ros2 launch rak_task_executor task_simulation.launch.py autostart:=false   # 只配置不激活

`task_executor` 现在是 `ManagedNode`（ADR-0020），所以这份夹具也照受管方式起它：
自己发 configure→activate 事件（与 `slam.launch.py` 同构）。留 `autostart:=false`
是为了手工验证"未激活的执行器不接受目标"。

这是软件夹具：fake Nav2 的反馈是命令运动估算，不是真实遥测，过绿不等于硬件验收
（`docs/hardware_acceptance.md`）。整机编排版在 `rak_bringup/supervisor.launch.py
profile:=fixture`。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory("rak_task_executor")
    waypoints = os.path.join(share, "config", "waypoints_sim.yaml")
    executor = LifecycleNode(
        package="rak_task_executor",
        executable="task_executor",
        name="task_executor",
        namespace="",
        output="screen",
        parameters=[{"waypoints_file": waypoints, "slow_planner": "local"}],
    )
    return LaunchDescription([
        DeclareLaunchArgument("goal_duration_sec", default_value="60.0"),
        DeclareLaunchArgument(
            "autostart", default_value="true",
            description="false = 只把执行器留在 unconfigured，用来验证未激活不接受目标",
        ),
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
        executor,
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(executor),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(LaunchConfiguration("autostart")),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=executor,
                start_state="configuring",
                goal_state="inactive",
                entities=[
                    EmitEvent(
                        event=ChangeState(
                            lifecycle_node_matcher=matches_action(executor),
                            transition_id=Transition.TRANSITION_ACTIVATE,
                        )
                    )
                ],
            ),
            condition=IfCondition(LaunchConfiguration("autostart")),
        ),
    ])
