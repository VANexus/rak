"""[AI] rak_wbc 启动入口（ADR-0011 + ADR-0020）。

只启动仲裁节点本身；底盘/臂的 ros2_control 由 rak_bringup（chassis_bringup.launch）或整机链路启动
负责。默认 enabled=false —— 启动后需显式调用 /wbc/enable 且现场安全确认后才发运动。

生命周期（ADR-0020 §3 单一所有者）：
  supervised:=false（默认，单独跑）  本文件自己发 configure→activate 事件，
                                    不会出现"进程在但 unconfigured"的静默节点
  supervised:=true （由 robot.launch.py 组合时）  不自启，转移交给 system_manager

`wbc_node` 是 `ManagedNode`：configure = 复合 URDF + HQP 求解器 + 图上接口；
activate = 100 Hz 仲裁环开始跑。两级都**不等于**运动许可 —— 发不发仍由
`enabled`（/wbc/enable）与底盘的 `allow_motion` 决定。

用法:
  ros2 launch rak_wbc wbc.launch.py
  ros2 launch rak_wbc wbc.launch.py supervised:=true   # 交给 system_manager
"""

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
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

_UNSUPERVISED = PythonExpression(
    ["'", LaunchConfiguration("supervised"), "' == 'false'"]
)


def generate_launch_description():
    parameters = get_package_share_directory("rak_wbc") + "/config/wbc_params.yaml"
    wbc = LifecycleNode(
        package="rak_wbc",
        executable="wbc_node",
        name="wbc",
        namespace="",
        output="screen",
        parameters=[parameters],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "supervised", default_value="false",
                description="true = 由 rak_bringup 的 system_manager 驱动转移，这里不自启",
            ),
            wbc,
            # 与 slam.launch.py 同构：configure 落到 inactive 之后再 activate。
            # 两个事件同时发会 races，结果就是"配置好了但没激活"——同样静默。
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(wbc),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ),
                condition=IfCondition(_UNSUPERVISED),
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=wbc,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(wbc),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                ),
                condition=IfCondition(_UNSUPERVISED),
            ),
        ]
    )
