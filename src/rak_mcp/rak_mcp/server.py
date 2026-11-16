"""标准 MCP 服务端（JSON-RPC 2.0 over stdio），工具清单来自工具注册表。

协议面只实现 MCP 规范里必需的那几件事：`initialize` / `notifications/initialized`
/ `tools/list` / `tools/call` / `ping`。载荷用换行分隔的 UTF-8 JSON，
stdout **只**写协议消息，日志一律走 stderr（否则客户端解析会炸）。

工具面不是另写一份菜单：`visible_tools()` 直接筛 `rak_tool_executor.tool_registry`，
所以这里新增/下线工具等于改工具表一处，MCP 侧自动跟随。
"""

from __future__ import annotations

import argparse
import json
import sys
from typing import Any, TextIO

from rak_tool_executor import tool_registry as reg

# 我们认识的协议版本，新→旧；客户端请求别的版本时回落到最新支持的。
SUPPORTED_PROTOCOL_VERSIONS = ("2025-06-18", "2025-03-26", "2024-11-05")
DEFAULT_PROTOCOL_VERSION = SUPPORTED_PROTOCOL_VERSIONS[0]
SERVER_NAME = "rak-robot"
SERVER_VERSION = "0.1.0"

PARSE_ERROR = -32700
INVALID_REQUEST = -32600
METHOD_NOT_FOUND = -32601
INVALID_PARAMS = -32602
INTERNAL_ERROR = -32603


class McpError(Exception):
    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


def visible_tools(allow_motion: bool) -> list[reg.ToolSpec]:
    """只暴露真的能用的工具；运动类是否露头由启动开关决定（与 harness 同一规则）。"""
    out = []
    for spec in reg.TOOLS:
        if spec.status != "implemented" or spec.deprecated:
            continue
        if spec.safety.startswith("motion") and not allow_motion:
            continue
        out.append(spec)
    return out


class RobotMcpServer:
    def __init__(self, bridge: Any, *, allow_motion: bool) -> None:
        self._bridge = bridge
        self._allow_motion = bool(allow_motion)

    # ------------------------------------------------------------- protocol
    def handle(self, message: Any) -> dict | None:
        """处理一条 JSON-RPC 消息；通知返回 None。"""
        if not isinstance(message, dict):
            return _error_response(None, INVALID_REQUEST, "message must be a JSON object")
        msg_id = message.get("id")
        method = message.get("method")
        params = message.get("params") or {}
        is_notification = "id" not in message
        if not isinstance(method, str):
            if is_notification:
                return None
            return _error_response(msg_id, INVALID_REQUEST, "missing 'method'")
        if is_notification:
            # notifications/initialized、notifications/cancelled 等一律无回包。
            return None
        try:
            result = self._dispatch(method, params)
        except McpError as exc:
            return _error_response(msg_id, exc.code, exc.message)
        except Exception as exc:  # noqa: BLE001 - 任何内部异常都要变成协议错误，不能哑掉
            return _error_response(msg_id, INTERNAL_ERROR, f"{type(exc).__name__}: {exc}")
        return {"jsonrpc": "2.0", "id": msg_id, "result": result}

    def _dispatch(self, method: str, params: Any) -> dict:
        if method == "initialize":
            return self._initialize(params)
        if method == "tools/list":
            return {"tools": [self._tool_definition(s) for s in
                              visible_tools(self._allow_motion)]}
        if method == "tools/call":
            return self._tools_call(params)
        if method == "ping":
            return {}
        raise McpError(METHOD_NOT_FOUND, f"method '{method}' not found")

    def _initialize(self, params: Any) -> dict:
        requested = params.get("protocolVersion") if isinstance(params, dict) else None
        version = (requested if requested in SUPPORTED_PROTOCOL_VERSIONS
                   else DEFAULT_PROTOCOL_VERSION)
        return {
            "protocolVersion": version,
            "capabilities": {"tools": {"listChanged": False}},
            "serverInfo": {"name": SERVER_NAME, "version": SERVER_VERSION},
            "instructions": (
                "本机机器人工具面。read 类直接可用；运动类（chassis.drive / task.* / arm.*）"
                "仅在服务端以 --allow-motion 启动时出现。运动都带速度上限与时长上限，"
                "到点自动置零。"
            ),
        }

    def _tool_definition(self, spec: reg.ToolSpec) -> dict:
        return {
            "name": spec.name,
            "description": spec.description,
            "inputSchema": spec.args_schema,
            "annotations": {"readOnlyHint": spec.safety in (reg.SAFETY_READ, reg.SAFETY_SPEAK)},
        }

    def _tools_call(self, params: Any) -> dict:
        if not isinstance(params, dict):
            raise McpError(INVALID_PARAMS, "tools/call params must be an object")
        name = params.get("name")
        arguments = params.get("arguments") or {}
        if not isinstance(name, str):
            raise McpError(INVALID_PARAMS, "tools/call requires string 'name'")
        if not isinstance(arguments, dict):
            raise McpError(INVALID_PARAMS, "'arguments' must be an object")
        if name not in {s.name for s in visible_tools(self._allow_motion)}:
            raise McpError(INVALID_PARAMS,
                           f"tool '{name}' is not exposed by this server")
        spec = reg.get_tool(name)
        outcome = self._bridge.call(name, arguments, spec.timeout_s)
        ok = bool(outcome.get("accepted")) and outcome.get("status") == "ok"
        return {
            "content": [{"type": "text",
                         "text": json.dumps(outcome, ensure_ascii=False)}],
            "isError": not ok,
        }

    # ---------------------------------------------------------------- stdio
    def serve(self, stdin: TextIO, stdout: TextIO) -> None:
        for line in stdin:
            line = line.strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError as exc:
                _write(stdout, _error_response(None, PARSE_ERROR, f"invalid JSON: {exc}"))
                continue
            response = self.handle(message)
            if response is not None:
                _write(stdout, response)


def _error_response(msg_id: Any, code: int, message: str) -> dict:
    return {"jsonrpc": "2.0", "id": msg_id, "error": {"code": code, "message": message}}


def _write(stdout: TextIO, payload: dict) -> None:
    stdout.write(json.dumps(payload, ensure_ascii=False) + "\n")
    stdout.flush()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog="rak_mcp_server",
        description="标准 MCP（stdio）工具面，转发到 rak_tool_executor。")
    parser.add_argument(
        "--allow-motion", action="store_true",
        help="暴露并放行运动类工具（chassis.drive / task.* / arm.*）。默认关闭。")
    parser.add_argument(
        "--name", default="rak-mcp",
        help="写进 ToolProposal.requested_by 的客户端标签（进审计）。")
    args = parser.parse_args(argv)

    from .bridge import ToolBridge

    bridge = ToolBridge(allow_motion=args.allow_motion, requested_by=args.name)
    server = RobotMcpServer(bridge, allow_motion=args.allow_motion)
    warm = bridge.warmup()
    print(f"[rak_mcp] tools={[s.name for s in visible_tools(args.allow_motion)]} "
          f"allow_motion={args.allow_motion} executor_online={bridge.executor_online()} "
          f"warmup={warm}",
          file=sys.stderr, flush=True)
    try:
        server.serve(sys.stdin, sys.stdout)
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
