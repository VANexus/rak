"""测试与脚本共用的生命周期驱动helper：把转移这件事收在一处。

存在的理由：rclpy 的 `LifecycleNode` **没有**进程内触发转移的公开 API —— 转移必须由
另一个节点经 `lifecycle_msgs/srv/ChangeState` 发起（生产里是 `system_manager`，测试里
也得是这样，否则测的就不是同一件事）。把这段样板放进包里，避免每个测试各自手写一遍
—— 手抄的 lifecycle 驱动代码是"节点起来了但没激活"这类假故障的常见来源。
"""

from __future__ import annotations

import time
from typing import Any

import lifecycle_msgs.msg
import lifecycle_msgs.srv
import rclpy
from rclpy.node import Node


class _Driver(Node):
    """只用来发转移与查状态；节点名刻意不像任何真实组件。"""

    def __init__(self) -> None:
        super().__init__("rak_lifecycle_test_driver")


def _await(future: Any, timeout: float) -> Any:
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.result() if future.done() else None


def state_of(driver: Node, node_fqn: str, timeout: float = 2.0) -> str | None:
    client = driver.create_client(lifecycle_msgs.srv.GetState, f"{node_fqn}/get_state")
    if not client.wait_for_service(timeout_sec=timeout):
        return None
    response = _await(client.call_async(lifecycle_msgs.srv.GetState.Request()), timeout)
    return response.current_state.label if response is not None else None


def transition(driver: Node, node_fqn: str, verb: str, timeout: float = 5.0) -> bool:
    client = driver.create_client(lifecycle_msgs.srv.ChangeState, f"{node_fqn}/change_state")
    if not client.wait_for_service(timeout_sec=timeout):
        return False
    request = lifecycle_msgs.srv.ChangeState.Request(
        transition=lifecycle_msgs.msg.Transition(
            id=getattr(lifecycle_msgs.msg.Transition, f"TRANSITION_{verb.upper()}"),
            label=verb,
        )
    )
    response = _await(client.call_async(request), timeout)
    if response is None or not response.success:
        return False
    goal = {"configure": "inactive", "activate": "active", "deactivate": "inactive",
            "cleanup": "unconfigured"}[verb]
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if state_of(driver, node_fqn) == goal:
            return True
        time.sleep(0.02)
    return False


def bring_up(driver: Node, node_fqn: str, timeout: float = 10.0) -> bool:
    """configure → activate，任一步失败即 False（调用方据此给出可读断言）。"""
    return transition(driver, node_fqn, "configure", timeout) and transition(
        driver, node_fqn, "activate", timeout
    )


def new_driver() -> Node:
    if not rclpy.ok():
        rclpy.init()
    return _Driver()
