# "想让机器人做一件工具菜单里的事"。这是提案,不是命令:
# 执行前必过确定性前置检查与护栏,任何一方可以拒绝。
string call_id          # uuid4;ToolResult 原样回环
string source           # harness(循环) | operator(人工直接注入,仅台架) | mcp(跨进程 MCP 客户端,ADR-0028)
string tool             # schema 注册表里的名字,如 "say" / "arm.home"
string args_json        # JSON object 字符串,过注册表校验
string requested_by     # 发起方的人类可读标签(操作员/session id),进审计
bool operator_confirmed # motion_* 类工具必须为 true(循环里人已确认);read/speak 忽略
string confirmed_by     # 确认人标签,进 detail_json 与语料
