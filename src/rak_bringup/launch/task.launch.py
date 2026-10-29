"""任务层单独一个可组合入口：执行器（受管）+ 可选 harness / tool 执行器。

为什么从 `task_navigation.launch.py` 里拆出来：`robot.launch.py` 的 `full` 档需要
**只**拉任务层（导航链它自己已经 include 过了），否则要么重复起一份 Nav2，要么
整个任务域起不来。拆开后 `task_navigation.launch.py` = nav2 + 本文件，语义不变。

生命周期（ADR-0020）：
  `/task_executor` 是 `ManagedNode` 且 owner=system_manager —— `supervised:=true` 时
  本文件不发转移，等 manager；`supervised:=false`（默认）时自己 configure→activate。
  `/harness`、`/tool_executor` 仍是 observed 普通节点：它们各自带着线程与子进程，
   managed 化要先把"可被打断"做对（见 ADR-0020 待闭合项 #1）。
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
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

_UNSUPERVISED = PythonExpression(
    ["'", LaunchConfiguration("supervised"), "' == 'false'"]
)


def generate_launch_description():
    share = get_package_share_directory("rak_bringup")
    waypoints_default = os.path.join(
        get_package_share_directory("rak_task_executor"), "config", "waypoints.yaml"
    )
    executor = LifecycleNode(
        package="rak_task_executor",
        executable="task_executor",
        name="task_executor",
        namespace="",
        output="screen",
        parameters=[{
            "slow_planner": LaunchConfiguration("slow_planner"),
            "llm_config_path": LaunchConfiguration("llm_config_path"),
            "llm_vendor": LaunchConfiguration("llm_vendor"),
            "waypoints_file": LaunchConfiguration("waypoints_file"),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("slow_planner", default_value="local",
                              choices=["local", "rai", "harness"]),
        DeclareLaunchArgument("llm_config_path", default_value=""),
        DeclareLaunchArgument("llm_vendor", default_value=""),
        DeclareLaunchArgument("waypoints_file", default_value=waypoints_default),
        DeclareLaunchArgument("harness", default_value="true",
                              choices=["true", "false"],
                              description="起 harness_node（ADR-0010 仲裁缝）"),
        DeclareLaunchArgument("tool_executor", default_value="false",
                              choices=["true", "false"],
                              description="起 tool_executor（手臂工具执行器）"),
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 转移交给 rak_bringup 的 system_manager，本文件不自启",
        ),
        executor,
        EmitEvent(
            event=ChangeState(
                lifecycle_node_matcher=matches_action(executor),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ),
            condition=IfCondition(_UNSUPERVISED),
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
            condition=IfCondition(_UNSUPERVISED),
        ),
        Node(
            package="rak_harness",
            executable="harness_node",
            name="harness",
            output="screen",
            condition=IfCondition(LaunchConfiguration("harness")),
        ),
        Node(
            package="rak_tool_executor",
            executable="tool_executor",
            name="tool_executor",
            output="screen",
            parameters=[{"allow_motion": False}],
            condition=IfCondition(LaunchConfiguration("tool_executor")),
        ),
    ])
