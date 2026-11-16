"""ROS 侧桥：一次 MCP 调用 = 一条 ToolProposal，等它的 ToolResult。

本模块刻意只做翻译，不做裁决：

- 工具表、参数校验、默认超时、速度上限全部来自
  `rak_tool_executor`（`tool_registry` / `chassis_limits`），MCP 层不复制任何一份；
- 权限、护栏、`allow_motion`、审计仍在 `rak_tool_executor` 节点里执行，
  这里是**跨进程的又一个客户端**，不是第二个执行器；
- 因此 MCP 客户端拿到的拒绝理由，和 harness 循环拿到的完全同源。

`source` 字段用契约里的新增值 `mcp`（ADR-0028）；真实客户端身份写进
`requested_by`，两者都进审计。执行器对 `mcp` 不做特殊放行——它和 harness 走同一套
护栏，区别只在审计里可区分来源。
"""

from __future__ import annotations

import json
import threading
import time
import uuid

import rclpy
from rak_interfaces.msg import ToolProposal, ToolResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from rak_tool_executor import tool_registry as reg


class ToolBridge:
    """把工具调用转发给 `/tool_proposals` 并回收 `/tool_results`。"""

    def __init__(
        self,
        *,
        allow_motion: bool = False,
        requested_by: str = "mcp",
        node_name: str = "mcp_bridge",
    ) -> None:
        self._allow_motion = bool(allow_motion)
        self._requested_by = requested_by
        self._pending: dict[str, ToolResult] = {}
        self._lock = threading.Lock()
        self._cv = threading.Condition(self._lock)
        self._closed = False

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(node_name)
        self._pub = self._node.create_publisher(ToolProposal, "/tool_proposals", 10)
        self._node.create_subscription(ToolResult, "/tool_results", self._on_result, 10)
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin, name="mcp-ros-spin", daemon=True)
        self._spin_thread.start()

    # ------------------------------------------------------------------ spin
    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001 - 退出阶段的 executor 异常不该拖垮 stdio 主循环
            pass

    def _on_result(self, msg: ToolResult) -> None:
        with self._cv:
            self._pending[msg.call_id] = msg
            # 有界：无人等待的残留结果（例如客户端已超时离开）最多留 64 条。
            while len(self._pending) > 64:
                self._pending.pop(next(iter(self._pending)))
            self._cv.notify_all()

    # ------------------------------------------------------------------ state
    def executor_online(self) -> bool:
        """执行器是否在听结果话题。未激活/未启动时这里就是 0。"""
        return self._node.count_subscribers("/tool_results") > 0

    def warmup(self, timeout_s: float = 3.0) -> bool:
        """开局做一次只读往返，把 DDS 发现跑完。

        为什么需要：MCP 客户端连上来就立刻发第一条 tools/call，此时"本进程的
        结果订阅"和"执行器的提案订阅"之间的发现可能还没完成，第一条提案的
        结果会丢（实测：首条 query 超时，第二条正常）。只读工具做一次预热是
        无害的；运动类不能靠重试兜底，所以这里必须在第一次运动之前完成。
        """
        if not self.executor_online():
            return False
        outcome = self.call("query.joint_states", {}, timeout_s)
        return outcome.get("status") in ("ok",)

    # ------------------------------------------------------------------- call
    def call(self, tool: str, args: dict, timeout_s: float) -> dict:
        """返回 {"status","accepted","reason","detail"}；绝不抛给调用方。"""
        spec = reg.get_tool(tool)
        if spec is None:
            return {"status": "rejected", "accepted": False,
                    "reason": f"unknown tool '{tool}'", "detail": {}}
        errors = reg.validate_args(spec, args)
        if errors:
            return {"status": "rejected", "accepted": False,
                    "reason": "args failed schema: " + "; ".join(errors),
                    "detail": {"schema": spec.args_schema}}
        if not self.executor_online():
            return {"status": "error", "accepted": False,
                    "reason": "tool_executor is not running (no /tool_results subscriber)",
                    "detail": {}}
        motion = spec.safety.startswith("motion")
        if motion and not self._allow_motion:
            return {"status": "rejected", "accepted": False,
                    "reason": ("motion tools are disabled for this MCP server; "
                               "restart it with --allow-motion after clearing the bench"),
                    "detail": {"safety": spec.safety}}

        call_id = uuid.uuid4().hex
        msg = ToolProposal(
            call_id=call_id,
            source="mcp",
            tool=tool,
            args_json=json.dumps(args, ensure_ascii=False),
            requested_by=self._requested_by,
            operator_confirmed=bool(motion and self._allow_motion),
            confirmed_by=self._requested_by if motion and self._allow_motion else "",
        )
        # 提案发出去之前确认至少有一个订阅者，避免"发了但没人收"变成一句超时。
        deadline = time.monotonic() + 1.0
        while self._pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        with self._cv:
            deadline_wait = timeout_s
            self._pub.publish(msg)
            got = self._wait_result(call_id, deadline_wait)
        if got is None:
            return {"status": "timeout", "accepted": False,
                    "reason": f"no ToolResult for {call_id} within {timeout_s:.1f}s",
                    "detail": {"call_id": call_id}}
        try:
            detail = json.loads(got.detail_json) if got.detail_json else {}
        except json.JSONDecodeError:
            detail = {"raw_detail": got.detail_json}
        return {"status": got.status, "accepted": bool(got.accepted),
                "reason": got.reason, "detail": detail, "call_id": got.call_id}

    def _wait_result(self, call_id: str, timeout_s: float) -> ToolResult | None:
        # 调用方已持有 self._cv 锁。
        self._cv.wait_for(lambda: call_id in self._pending, timeout=timeout_s)
        return self._pending.pop(call_id, None)

    # ------------------------------------------------------------------ close
    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._executor.shutdown()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
