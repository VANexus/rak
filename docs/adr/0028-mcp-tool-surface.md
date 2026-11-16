# ADR-0028: 跨进程 MCP 工具面 —— 同一提案缝的第三个客户端

- 状态: 已接受（**mock 链路已验；真机链路未验**，见「待闭合项」）
- 日期: 2026-11-16
- 相关: `src/rak_mcp/`（MCP 服务、桥、台架 launch）· `src/rak_tool_executor/`（工具表、护栏、审计）·
  `src/rak_interfaces/msg/ToolProposal.msg`（`source=mcp`）·
  `docs/design/brain_arm_tools.md`（D2/§3.1/§9 原文已按本篇修订）· ADR-0020（受管生命周期与运动许可）

## 背景

`brain_arm_tools.md` 的 D2 原本写死"控制面是机器人自己的循环，不设 MCP 之类的外部控制面"。
2026-11-16 的需求反过来：要让**外部智能体/客户端**按标准 MCP 调用本机机器人栈
（下发速度、读状态、取传感器摘要、触发行为），且明确"走标准协议、不自己发明工具协议"。

当时的硬约束：

1. **不许出现第二套事实**。工具清单、参数 schema、默认超时、速度上限已经各只有一处定义
   （`tool_registry.py` / `chassis_limits.py`），MCP 层不得抄一份。
2. **不许出现第二个执行器**。权限、护栏、审计只在 `rak_tool_executor` 里判；
   越权路径开一个就等于开全部。
3. **运动不许"调用后一直跑"**。既有底盘控制器有 `reference_timeout` 兜底，但那是第二道闸，
   工具面自己必须有速度上限与时长上限。
4. 感知/规划线并行推进，工具面要能容纳后续接进来的能力，但不替它做设计。

盘点结论（决定了本篇的形状）：`ToolProposal`/`ToolResult` 契约、单写者执行器、stdlib-only
的工具表已经构成了一条干净的缝，MCP 只需要是**缝上的又一个客户端**。

## 决策

1. **新增独立包 `src/rak_mcp`（ament_python）**，不塞进 `rak_tool_executor`。
   执行器保持"只有提案入、只有结果出"，不为外部协议增加分支。
2. **协议面**：JSON-RPC 2.0 over stdio（MCP 标准传输），实现 `initialize` /
   `notifications/initialized` / `tools/list` / `tools/call` / `ping`。stdio 上只写协议帧，
   日志一律 stderr。不引第三方 SDK（本机无 `mcp` 包；纯 stdlib 实现，协议帧可单测）。
3. **工具清单 = 注册表**。`tools/list` 直接筛 `tool_registry.TOOLS`：
   只露 `status=implemented` 且未废弃的；`safety` 以 `motion` 开头的仅在服务端
   `--allow-motion` 时出现。`inputSchema` 原样用注册表里的 JSON Schema，
   `timeout_s` 用 `ToolSpec.timeout_s`（本轮新增字段，默认 30 s）。
4. **调用路径 = 提案**。`tools/call` → 本地 `validate_args` 预检 → 发 `ToolProposal`
   （`source=mcp`，`requested_by` 写客户端标签）→ 按 `call_id` 等 `ToolResult` → 
   包成 MCP `content`/`isError`。MCP 侧没有任何执行动作。
5. **`ToolProposal.source` 新增 `mcp`**（additive，`operator|harness` 语义不变）。
   执行器对 `mcp` 不做特殊放行，区别只在审计里可区分来源。
6. **底盘运动补三个工具**（否则闭环无从验证）：`chassis.drive`（带速度/时长上限）、
   `chassis.stop`（只写零）、`query.chassis_state`（轮速 + 里程计）。
7. **底盘单一写者变成可执行的互锁**：`chassis.drive` 在图上出现 Nav2 节点
   （`bt_navigator`/`controller_server`/`planner_server`/`velocity_smoother`，可配）时
   **结构化拒绝**（`detail.interlock="nav2_active"` + 挡路的节点名），
   而不是靠文档约定。`nav2_interlock:=false` 可关，属显式越权动作。
8. **`chassis.stop` 绕过 `allow_motion` 但必留审计**：它只写零、永不产生位移，
   卡在许可门上只会更危险；每次绕过写 `kind="tool_override"` 行
   （`source` / `requested_by` / `t` / 原因）。互锁同样不拦它——急停不该被互锁卡住。

## 理由

- **协议照抄，执行不照抄**：MCP 是"怎么说"，机器人栈是"能不能做"。把两者分开后，
  新增一个客户端（MCP、语音循环、调试 TUI）都不需要碰执行器。
- **注册表即菜单**：`tools/list`、参数校验、超时、速度上限全部 import 同一份，
  "MCP 里 0.3、执行器里 0.2"这种漂移在结构上不可能发生。
- **在场判定优于"正在导航"判定**：Nav2 是随时可以起速度的写者，空闲时也在场；
  读图零成本、不依赖 action 反馈。代价是 Nav2 起了但没跑时也挡底盘工具——压在安全侧。
- **急停不该有门**：所有许可都是为了"防止意外运动"，而零速命令本身不产生运动。

## 后果

**正面**

- 任何说 MCP 的客户端（Claude Desktop/Code、自研 agent）零适配即可列工具、读状态、下发低速运动。
- 底盘速度上限、时长上限、Nav2 互锁变成**可测**的三条闸（schema 拒绝 / 到点置零 / 结构化拒绝）。
- 审计里能分清 `harness` / `operator` / `mcp` 三个来源。

**负面**

- **多进程边界**：DDS 发现慢时首条调用可能丢（实测首条 query 超时）。
  现在靠开局一次只读预热（`warmup()`）兜；这是补丁，把握手时序彻底定下来要更多工作。
- 底盘工具与 Nav2 共用 `/mecanum_drive_controller/reference`，互锁只覆盖**同一执行器内**的
  并发；其余直连该话题的台架工具（`chassis_tui`、`chassis_key_test`）不受它约束。
- `chassis.stop` 的只读级自动放行意味着任何能发提案的进程都能写零速；
  恶意/错误来源可以打断导航任务。可接受（零速是安全侧），但要知道这是信用的边界。
- MCP 客户端在 stdio 上跑，**必须由客户端拉起**，不能作为 launch 的常驻节点；
  部署文档要写清这一点。
- 纯 stdlib 实现 MCP 只覆盖规范必需子集（没有 resources/prompts/sampling/进度通知）。

## 被否决的方案

1. **在 `rak_tool_executor` 里内建 MCP server**：把外部协议栈塞进安全节点，节点要同时
   负责 stdio 帧与运动护栏，错误域混在一起；也让"执行器不依赖任何客户端"这条失效。
2. **MCP 直接发 `TwistStamped` 到 `/mecanum_drive_controller/reference`**：
   绕过提案缝等于绕过权限、审计与互锁，且会造出第二个"速度上限写在哪"的答案。
3. **给 MCP 单独抄一份工具 schema**：短期省事，长期必然与注册表漂移（本仓已有
   `query.joints` vs `query.joint_states` 这类历史漂移）。
4. **用 rosbridge + 自研封装充当 MCP**：既要额外守护 websocket，又仍然需要自己发明
   "工具"语义；与"走标准协议"的要求相反。
5. **只在文档里约定底盘单写者**：正是 2026-10-15 草案的做法，实践上没人执行；
   现在是代码级拒绝。

## 待闭合项

1. **真机链路未验（2026-11-16 冻结）**：mock 底盘（`mock_components/GenericSystem`）上已验证
   `tools/list → chassis.drive → query.chassis_state → chassis.stop` 与到点置零；
   **真实控制器 + 真轮子**下的位姿变化与停止漂移**尚未测** —— 用户当时正在测试底盘，
   明确要求所有人不要启动 `controller_manager`/CANopen 主站、不要往 `can0` 发帧。
   放行后第一步：直接对场上的 `/mecanum_drive_controller/reference` 跑一次
   `chassis.drive(0.05 m/s, 1.0 s)` + `query.chassis_state` + `chassis.stop`，
   记录位移量级与停止后 3 s 漂移。
   旁证（只读图查询，未加载控制器、未发帧）：真实 `controller_manager` 在场时
   `ros2 control list_controllers` 返回 "No controllers are currently loaded!"，
   即当时的真机链路里 **mecanum 控制器并未加载**，所以"能用"的前提不只是网络连通。
2. **首帧发现**：`warmup()` 是补丁。彻底做法是 MCP 侧在 `initialize` 前等待
   `/tool_results` 双向发现完成，并把等待时间写进 stderr 供客户端配置参考。
3. **互锁覆盖面**：`chassis_tui` / `chassis_key_test` 直连同一话题，不受互锁；
   是否把这些台架工具也纳管，等下一个真实"双写者"事故再定。
4. **`mcp` source 的下游消费**：`rak_harness`/语料合并侧目前只认 `harness|operator`，
   需要一次确认它对未知 `source` 是"记录不解释"而不是丢弃。
5. **真机验收指标**：`chassis.drive` 的方向/量级误差、停止后 3 s 位移漂移门限，需要在
   `docs/reference/hardware_acceptance.md` 补一条。**本轮没有实测数值**（真机验证被冻结），
   等放行后先测再写门限，不拿估算值占位。
