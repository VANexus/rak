"""纯协议测试：MCP 帧处理与工具表映射。不需要 ROS（bridge 只在 main 里导入）。"""

from rak_mcp.server import RobotMcpServer, visible_tools
from rak_tool_executor import tool_registry as reg


class FakeBridge:
    def __init__(self):
        self.calls = []

    def call(self, tool, args, timeout_s):
        self.calls.append((tool, args, timeout_s))
        return {"status": "ok", "accepted": True, "reason": "ok", "detail": {"echo": args}}


def _server(allow_motion=False, bridge=None):
    return RobotMcpServer(bridge or FakeBridge(), allow_motion=allow_motion)


def test_initialize_negotiates_known_version():
    resp = _server().handle({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                             "params": {"protocolVersion": "2025-06-18"}})
    assert resp["result"]["protocolVersion"] == "2025-06-18"
    assert resp["result"]["capabilities"]["tools"]["listChanged"] is False


def test_initialize_falls_back_on_unknown_version():
    resp = _server().handle({"jsonrpc": "2.0", "id": 1, "method": "initialize",
                             "params": {"protocolVersion": "1999-01-01"}})
    assert resp["result"]["protocolVersion"] == "2025-06-18"


def test_initialized_notification_has_no_response():
    assert _server().handle({"jsonrpc": "2.0", "method": "notifications/initialized"}) is None


def test_tools_list_matches_registry_when_motion_hidden():
    resp = _server(allow_motion=False).handle(
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
    names = [t["name"] for t in resp["result"]["tools"]]
    expected = [s.name for s in reg.TOOLS
                if s.status == "implemented" and not s.deprecated
                and not s.safety.startswith("motion")]
    assert names == expected
    assert "chassis.drive" not in names
    assert "query.chassis_state" in names


def test_tools_list_exposes_motion_when_allowed():
    resp = _server(allow_motion=True).handle(
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list"})
    names = [t["name"] for t in resp["result"]["tools"]]
    assert "chassis.drive" in names
    schema = {t["name"]: t["inputSchema"] for t in resp["result"]["tools"]}["chassis.drive"]
    assert schema["properties"]["vx_mps"]["maximum"] == 0.30


def test_unimplemented_tools_never_listed():
    names = [s.name for s in visible_tools(allow_motion=True)]
    assert "gripper.set" not in names
    assert "arm.reach" not in names


def test_tools_call_forwards_and_wraps_result():
    bridge = FakeBridge()
    resp = _server(allow_motion=False, bridge=bridge).handle(
        {"jsonrpc": "2.0", "id": 3, "method": "tools/call",
         "params": {"name": "query.chassis_state", "arguments": {}}})
    assert bridge.calls[0][0] == "query.chassis_state"
    assert resp["result"]["isError"] is False
    assert resp["result"]["content"][0]["type"] == "text"


def test_tools_call_hidden_tool_is_invalid_params():
    resp = _server(allow_motion=False).handle(
        {"jsonrpc": "2.0", "id": 4, "method": "tools/call",
         "params": {"name": "chassis.drive", "arguments": {"duration_s": 1.0}}})
    assert resp["error"]["code"] == -32602


def test_unknown_method_is_method_not_found():
    resp = _server().handle({"jsonrpc": "2.0", "id": 5, "method": "resources/list"})
    assert resp["error"]["code"] == -32601


def test_ping():
    assert _server().handle({"jsonrpc": "2.0", "id": 6, "method": "ping"})["result"] == {}
