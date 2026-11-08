"""Agent bridge: spawns and supervises the Claude Agent SDK loop subprocess.

The SDK's dependency tree (pydantic v2 and friends) must never enter the
ROS system Python, so the loop runs as a child process started with a
PYTHONPATH overlay that points only at the vendored SDK directory
(resolved by rak_harness.sdk: param -> RAK_HARNESS_SDK_DIR -> ament -> source tree;
never a hardcoded home directory). The bridge carries:

  * transcripts  (parent -> child, from /speech/transcript)
  * tool proposals (child -> parent -> /tool_proposals, source=harness)
  * tool results  (/tool_results -> parent -> child)
  * thinking / final text (child -> parent -> /harness/trace, /speech/say)

If the child reports down (SDK missing, key missing, crash), the bridge
notifies the node so the dialogue engine keeps the voice path alive.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import threading
from typing import Callable

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rak_interfaces.msg import ToolProposal, ToolResult


class AgentBridge:
    def __init__(
        self,
        node: Node,
        *,
        sdk_path: str,
        model: str,
        allow_motion_tools: bool,
        trace_pub,
        say: Callable[[str], None],
        on_state: Callable[[str, str], None],
        context_provider: Callable[[], dict] | None = None,
    ) -> None:
        self._node = node
        self._sdk_path = sdk_path
        self._model = model
        self._allow_motion_tools = allow_motion_tools
        self._trace = trace_pub
        self._say = say
        self._on_state = on_state
        self._context_provider = context_provider
        self._lock = threading.Lock()
        self._proc: subprocess.Popen | None = None
        # close() 置位：收尾线程据此不再回报 "down"（那是我们自己关的，不是故障）。
        self._stopping = False
        self._thread: threading.Thread | None = None
        self._proposal_pub = node.create_publisher(ToolProposal, "/tool_proposals", 10)
        self._result_sub = node.create_subscription(
            ToolResult, "/tool_results", self._on_tool_result, 10)
        self._transcript_sub = node.create_subscription(
            String, "/speech/transcript", self._on_transcript, 10)
        self._spawn()

    # ------------------------------------------------------------- lifecycle

    def _spawn(self) -> None:
        if self._stopping:
            return
        env = dict(os.environ)
        env["PYTHONPATH"] = os.pathsep.join(
            [self._sdk_path] + ([env["PYTHONPATH"]] if env.get("PYTHONPATH") else [])
        )
        # Isolated Claude config: the robot agent must not inherit the
        # operator's ~/.claude plugins/hooks (a SessionEnd hook once needed
        # node and hard-failed the session).
        try:
            config_dir = os.path.join(self._sdk_path, "claude-robot-config")
            os.makedirs(config_dir, exist_ok=True)
            env["CLAUDE_CONFIG_DIR"] = config_dir
        except OSError:
            pass
        # Custom model slot: the CLI validates model IDs against an Anthropic
        # whitelist; these env vars register a pass-through option so DeepSeek's
        # compat endpoint can receive its own model name (see probe notes).
        env.setdefault("ANTHROPIC_CUSTOM_MODEL_OPTION", self._model)
        env.setdefault("ANTHROPIC_CUSTOM_MODEL_OPTION_NAME", self._model)
        argv = [sys.executable, "-m", "rak_harness.agent_loop", "--model", self._model]
        if self._allow_motion_tools:
            argv.append("--allow-motion-tools")
        try:
            self._proc = subprocess.Popen(
                argv, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                text=True, bufsize=1, env=env,
            )
        except OSError as exc:
            self._on_state("down", f"agent subprocess spawn failed: {exc}")
            return
        self._thread = threading.Thread(
            target=self._pump, name="harness-agent", daemon=True)
        self._thread.start()
        self._node.get_logger().info(
            f"agent loop spawned (model={self._model}, motion_tools={self._allow_motion_tools})")

    def _pump(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        try:
            for line in proc.stdout:
                line = line.strip()
                if not line:
                    continue
                try:
                    self._handle(json.loads(line))
                except ValueError:
                    continue
        except (OSError, ValueError):
            pass
        if not self._stopping:
            self._on_state("down", "agent subprocess exited")

    def close(self) -> None:
        """停掉子进程与读线程；`deactivate → activate` 靠它保证不泄漏子进程。

        幂等、异常安全：任何一步失败都只是少收一次 SIGTERM，不影响调用方的转移。
        """
        self._stopping = True
        proc, self._proc = self._proc, None
        if proc is not None:
            try:
                if proc.stdin:
                    proc.stdin.write(json.dumps({"type": "shutdown"}) + "\n")
                    proc.stdin.flush()
            except (OSError, ValueError):
                pass
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                try:
                    proc.terminate()
                    proc.wait(timeout=2.0)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        proc.kill()
                    except OSError:
                        pass
            except OSError:
                pass
            for stream in (proc.stdin, proc.stdout):
                try:
                    if stream is not None:
                        stream.close()
                except (OSError, ValueError):
                    pass
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            try:
                thread.join(timeout=2.0)
            except RuntimeError:  # 从自己的线程里 join 自己
                pass
        for entity, destroy in (
            (self._result_sub, self._node.destroy_subscription),
            (self._transcript_sub, self._node.destroy_subscription),
            (self._proposal_pub, self._node.destroy_publisher),
        ):
            if entity is None:
                continue
            try:
                destroy(entity)
            except Exception:  # noqa: BLE001 - 节点可能已在销毁
                pass
        self._result_sub = None
        self._transcript_sub = None
        self._proposal_pub = None

    def destroy(self) -> None:
        self.close()

    # ------------------------------------------------------------- child msgs

    def _handle(self, msg: dict) -> None:
        kind = str(msg.get("type", ""))
        if kind == "status":
            self._on_state(str(msg.get("state", "?")), str(msg.get("detail", "")))
            self._trace_line("agent_status", msg)
        elif kind == "thinking":
            self._trace_line("thinking", msg)
        elif kind == "assistant":
            self._trace_line("assistant", msg)
        elif kind == "tool_call":
            self._trace_line("tool_call", msg)
        elif kind == "tool_result":
            self._trace_line("tool_result", msg)
        elif kind == "final":
            text = str(msg.get("text", "")).strip()
            if text:
                self._say(text)
            self._trace_line("final", msg)
        elif kind == "propose":
            self._publish_proposal(msg)

    def _publish_proposal(self, msg: dict) -> None:
        if self._stopping or self._proposal_pub is None:
            return
        proposal = ToolProposal(
            call_id=str(msg.get("call_id", "")),
            source="harness",
            tool=str(msg.get("tool", "")),
            args_json=str(msg.get("args_json", "{}")),
            requested_by="harness-agent",
            operator_confirmed=self._allow_motion_tools,
            confirmed_by="auto-confirm param" if self._allow_motion_tools else "",
        )
        self._proposal_pub.publish(proposal)

    # ------------------------------------------------------------- parent msgs

    def _on_transcript(self, msg: String) -> None:
        text = msg.data.strip()
        if self._stopping or not text or self._proc is None or self._proc.stdin is None:
            return
        payload_lines = []
        if self._context_provider is not None:
            try:
                context = self._context_provider()
            except Exception:
                context = {}
            if context:
                payload_lines.append(json.dumps({"type": "context", **context},
                                                ensure_ascii=False))
        payload_lines.append(json.dumps({"type": "transcript", "text": text},
                                        ensure_ascii=False))
        with self._lock:
            try:
                self._proc.stdin.write("\n".join(payload_lines) + "\n")
                self._proc.stdin.flush()
            except OSError:
                self._on_state("down", "agent stdin closed")

    def _on_tool_result(self, msg: ToolResult) -> None:
        if self._stopping or self._proc is None or self._proc.stdin is None:
            return
        try:
            detail = json.loads(msg.detail_json) if msg.detail_json else {}
        except ValueError:
            detail = {"raw": msg.detail_json}
        payload = json.dumps({
            "type": "tool_response", "call_id": msg.call_id,
            "status": msg.status, "reason": msg.reason, "detail": detail,
        }, ensure_ascii=False)
        with self._lock:
            try:
                self._proc.stdin.write(payload + "\n")
                self._proc.stdin.flush()
            except OSError:
                self._on_state("down", "agent stdin closed")

    def _trace_line(self, label: str, msg: dict) -> None:
        if self._trace is None or self._stopping:
            return
        out = String()
        out.data = json.dumps({"label": label, **msg}, ensure_ascii=False)
        self._trace.publish(out)
