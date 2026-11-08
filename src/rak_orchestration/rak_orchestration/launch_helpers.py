"""launch 侧的生命周期助手（ADR-0020 §5 的 launch 半边）。

**问题**：把一个节点改成 `LifecycleNode` 之后，launch 里如果还写 `Node(...)`，
进程起来了但永远停在 `unconfigured` —— 订阅不建、话题不发、也不报错。
这是本仓库记录过的最贵的故障形态，所以"改节点"和"改 launch"必须同时发生。

**做法**：每个 launch 统一走 `managed(...)`：

* 返回 `LifecycleNode` + 两个事件处理器（configure → activate）；
* `supervised:=true`（由 `system_manager` 驱动）时**不**自启，交给编排层；
* `supervised:=false`（默认，单独跑某个 launch 排障）时自启，
  避免出现"进程在但不干活"的静默节点。

为什么这一层值得单独存在：没有它，每个 launch 都要抄一遍
`OnProcessStart + OnStateTransition` 六行样板，抄漏一处就多一个静默节点，
而且**抄漏不会报错**。门禁只能检查"用了 LifecycleNode"，检查不了"事件处理器写全了"，
所以正确性必须来自"只允许一种写法"。
"""

from __future__ import annotations

from typing import Any, Sequence

from launch import Action
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

#: AST 门禁认识的助手名；改这里必须同步改 test_system_manifest._node_actions。
HELPER_NAMES = ("managed",)


def managed(
    *,
    package: str,
    executable: str,
    name: str,
    parameters: Sequence[Any] | None = None,
    remappings: Sequence[tuple[str, str]] | None = None,
    namespace: str = "",
    output: str = "screen",
    autostart: Any = None,
    ros_arguments: Sequence[str] | None = None,
) -> list[Action]:
    """起一个受管节点，并按 `supervised` 决定是否自启到 active。

    ``autostart``：
      * ``None``（默认）—— 受 ``supervised`` 开关控制（它由 launch 声明，默认 false）；
      * ``True`` / ``False`` —— 硬编码自启与否（夹具、单节点测试用）。
    """
    node = LifecycleNode(
        package=package,
        executable=executable,
        name=name,
        namespace=namespace,
        output=output,
        parameters=list(parameters or []),
        remappings=list(remappings or []),
        ros_arguments=list(ros_arguments or []),
    )
    condition = _autostart_condition(autostart)
    if condition is False:
        return [node]
    return [
        node,
        RegisterEventHandler(
            OnProcessStart(
                target_action=node,
                on_start=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=Transition.TRANSITION_CONFIGURE,
            ))],
            ),
            condition=condition,
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=node,
                start_state="configuring",
                goal_state="inactive",
                entities=[EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(node),
                    transition_id=Transition.TRANSITION_ACTIVATE,
                ))],
            ),
            condition=condition,
        ),
    ]


def _autostart_condition(spec: Any):
    """把 `autostart` 规格翻译成 launch 条件；返回 False 表示"一个处理器都不要"。"""
    if spec is True:
        return None
    if spec is False:
        return False
    if spec is None:
        return UnlessCondition(LaunchConfiguration("supervised"))
    return UnlessCondition(spec)
