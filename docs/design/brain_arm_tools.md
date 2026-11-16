# 大脑 × 机械臂 × 工具：主动智能操作层设计

> 状态：**DESIGN DRAFT**（2026-10-15）。未接受；接受时按 `docs/adr/README.md` 流程落 ADR 并把本文件状态改为"已由 ADR-xxxx 接受"。
> 事实源：`docs/reference/arm_moveit_contract.md`（臂契约 FROZEN）、`docs/adr/0007`（ros2_control 目标）、`docs/adr/0010`（Jev/Harness 仲裁缝）、`docs/adr/0011`（WBC）、`docs/how-to/laya_tensorrt_runbook.md`（Laya 运行时）、`src/rak_chassis` 实测（2026-10-14）。
> 语言约定：中文叙述、英文标识符。度量单位 SI。

## 0. 目标与非目标

**目标**

- G1 让决策侧（操作员、未来 Harness/S2）能通过**一条缝、一个契约**安全地操作机械臂（IK 位姿到位）与现成工具（导航任务、语音、夹爪、状态查询）。
- G2 所有主动动作可审计、可回放，并**顺带长出带标注的语料**（服务后续 Laya 微调）。
- G3 不破坏任何冻结契约：`types.py` / `Action`+`Action.msg` / 臂契约 / ADR-0010 的方向（brain 只发决策、Harness 仲裁、执行器执行）。
- G4 单写者与安全权威格局不变：`guard()` 仍是唯一停止权威；WBC 闭合前臂走 `arm_controller`、底盘走 Nav2，永不并发写。

**非目标**

- 不让 Laya 直接输出关节角或末端位姿（它是 7 类文本分类器，见 §2 D1）。
- 不新增绕过 ros2_control / WBC 的速度或关节命令通道。
- 不在本仓内建训练管线（语料是交付物，训练在仓外）。
- 不解决急停/舵机门控等硬件缺口本身（列为门，见 §9）。

## 1. 现状事实（设计依据，均为实测/冻结项）

| 能力 | 现状 | 出处 |
|---|---|---|
| 底盘 | 4 轮麦克纳姆，`mecanum_drive_controller` 吃 `/reference`（vx/vy/wz），0.5 s 无输入自动置零 | `chassis_controllers.yaml` |
| 臂 | 6 舵机 `j1_base…j6_wrist_roll`，位置域 0…270°（0…4.71239 rad），0.1° 量化，≤50 Hz；**反馈=指令回声（无编码器）**；`arm_controller`（FollowJointTrajectory）已实车验证 | 臂契约 §1/§2、AGENTS.md 2026-10-14 |
| 夹爪 | 第 7 路命令协议预留（GRIP），**无物理通道，回读恒 0** | 臂契约 §2.3 |
| 急停 | **只停轮驱，舵机保位** | 臂契约 §2.5 |
| IK | `rak_arm_pinocchio_ik` 提供 `/solve_ik`（moveit_msgs/GetPositionIK，pinocchio DLS）；**默认参数是六轴仿真 fixture 的关节名**，可用 `urdf_path`/`joint_names`/`tip_frame` 参数化 | `ik_service_node.py` |
| 抓取感知 | GraspNet 候选（`GraspCandidates`，相机系，+X 接近轴）+ `SelectGrasp.srv` 选择；RViz viewer/planner 已有 | `src/rak_grasp_perception` |
| 导航/任务 | `rak_task_executor`：`ExecuteTask`（文本/航点 + 可选 PoseStamped）、`TaskControl`（PAUSE/RESUME/CANCEL），Nav2 目标唯一所有者 | ADR-0007 待闭合 #7 语义 |
| 语音 | `/speech/say`（Say 服务）→ TTS → 喇叭；ASR→`rak_intent` 文本意图 | `src/rak_speech` |
| WBC | 骨架：`/cmd_vel`+`/wbc/ee_target` → 仲裁 → 底盘 reference + `/wbc/arm_setpoints`；`publish_arm=false`，默认门关 | ADR-0011 |
| Laya | S1 进程内 7 类文本决策，实测域外（与几何基线一致率 0%，p≈0.24）；`/brain/decision` 今天**无执行器消费者**（ADR-0010：唯一既定消费者是未来的 Harness，且只作上下文） | 昨日探针、ADR-0010 |
| 大脑输出契约 | `ActionKind` 7 类 FROZEN；`state_text` FROZEN（2317 行语料已存在） | `types.py`、ADR-0013 |

## 2. 核心设计决定

**D1 — Laya 永不直接操作工具。**
S1 保持"场景文本 → 7 类策略标签"的快路径位：它对臂/工具的参与只有两种合法形式——(a) `gate()` 的升级层级影响提案被允许的速度/范围（低置信 → 提案只能以最低速执行或直接拒绝）；(b) 未来通过 `SceneState.flags` 新增**值**（如 `arm_moving`、`gripper_holding`）让 Laya 在文本输入里"看得见"臂态——flags 是既有格式字段，加值不改 `to_text()` 格式，不毁语料。臂/工具的"做什么、做哪里"属于 S2/操作员的结构化提案，不是 7 类标签能表达的（扩标签=破冻结契约+引擎重建，见 §7，v2 再议）。

**D2 — 单一提案缝；消费这条缝的不是控制台，是 agent 循环本身。**
所有主动动作是一个版本化提案契约（§3）。工具的消费者是**像 Claude Code 一样的内部循环**：
LLM 以 function-calling 调菜单（JSON Schema 即工具定义），提案出、结果回灌上下文、循环继续。
"操作台"就是对话（语音进/出 + 开发台文本），**不是 CLI**。跨进程 MCP 工具面由
**ADR-0028** 单独接受（2026-11-16 修订 D2 的"不设 MCP"部分）：它是同一条提案缝的
第三个客户端，不是第二个执行器。
提案 source 三值：`harness`（循环，含操作员在环的监督会话）、`operator`（人工直接注入，
仅调试/commissioning）、`mcp`（跨进程 MCP 客户端，ADR-0028；真实身份写 `requested_by` 进审计）。
brain 依旧只发 `/brain/decision`（ADR-0010：决策是上下文，不是命令）；
执行侧只有一个节点 `rak_tool_executor`（新包，§10）。

**D3 — 冻结契约一律不改，新动作用新消息。**
`ActionKind`/`Action.msg`/`state_text` 不动。工具动作定义在新消息里（§3）。语料侧以新增事件行（`kind="tool_call"`/`"tool_result"`）入账——`Logger.log(**kwargs)` 是追加式，不碰已有字段。

**D4 — IK 走轻量直达路径（P2），MoveIt 全量规划留给抓取（P3）。**
P2 链路：提案 → 前置检查 → `/solve_ik`（pinocchio）→ 确定性时间参数化 → `arm_controller` FollowJointTrajectory。理由：内存（Jetson 上 move_group + 规划场景是大头）、确定性（直线插值+恒速，可预测到不可预测的东西旁边做什么）、以及 P2 只做自由空间单点到位，没有碰撞规划需求。P3 抓取必须引入点云碰撞时再上 MoveIt 规划（消费同一臂契约，方向合规）。**IK 服务必须以真臂参数启动**：`urdf_path=rak_arm.urdf.xacro`（xacro 展开）、`joint_names=[j1_base…j6_wrist_roll]`（契约 §1）、`tip_frame=tool0`；**绝不使用 fixture 默认关节名**（六轴 fixture 永不碰硬件，AGENTS.md 铁律）。

**D5 — 单写者。**
P2–P3：臂只写 `arm_controller`（+ 夹爪位置话题），底盘只归 Nav2；`arm.reach` 执行期间工具执行器**拒绝** `task.goto`（安全第一，P4 由 WBC 仲裁后才放开并发）。WBC 闭合（ADR-0011 待闭合项）后，`scene.scan`/全身动作走 WBC，提案契约不变。

**D6 — 硬件缺口显式成门。**
夹爪无物理通道（工具可调用但返回 `rejected` 并说明原因，直到实测接通）；急停不停臂 → 臂工具默认速度上限取 URDF 限位的 25%，并叠加臂工作空间护栏（§6）；无编码器 → 所有"成功"语义都是**指令按回读达标**，不是物理确认，抓取必须准静态。

**D7 — 循环用现成 harness SDK：Claude Agent SDK（官方 Python），不自研循环。**
调研结论（2026-10-15，后经操作员裁定改为采用）：现成 harness 首选 **Claude Agent SDK**
（`claude-agent-sdk`，Anthropic 官方，Python，内嵌 Claude Code 运行时：in-process 自定义工具
`@tool` + SDK MCP server、`canUseTool` 权限回调、hooks、allowed/disallowed、sessions）。
选它的核心理由：它把"agent loop 的机制"（消息编排、tool_use/tool_result 回灌、上下文管理、
重试、轮次/成本上限）全部实现且是 Claude Code 同款——本设计要的就是这个 loop 本身，
自研 ~150 行只是把同样的事写两遍。已知代价与纪律：

1. **端点兼容是 P1 第一探针**（§9 #10）：Harness 走 Anthropic 兼容端点（ADR-0010，
   DeepSeek），SDK 官方只承诺 Bedrock/Vertex/Foundry 三类第三方认证；tool_use 往返
   不通过就换官方端点或退回 OpenAI 兼容格式（loop 侧只换传输层配置）。
2. **权限两层的分工写死**：SDK 的 `canUseTool`/allowed_tools 只做 **UX 与粗粒度闸**
   （哪些工具可见、motion 类统一走人确认）；**权威仍在 `rak_tool_executor`**
   （确定性前置检查/护栏/审计，§6）——工具处理器本身既是 SDK 工具也是执行器的唯一客户端，
   两条腿走同一份 schema 表。
3. **资源纪律**：SDK 内嵌 Claude Code CLI 子进程（Node 系）；Jetson 4 GB、桌面会话下典型
   可用 ~575 MiB。harness_node 为低优先级进程、headless 下运行、可整体禁用
   （`enable:=false` 时机器人其余功能全不受影响）。
4. **装箱**：user-site 安装（同 `tokenizers` 先例：无 rosdep key、文档注明），
   harness 的 import 失败必须只禁音 loop，不拖垮任何其他节点。
5. 原方案（自研循环 + `anthropic` SDK 裸 messages 循环）作为探针失败时的退路记录在案，
   工具箱/契约/执行器不变，只换 loop 实现。

**何时换框架**：菜单长到 >~20 工具或需子代理/移交编排时重议。

## 3. 契约（P0 新增，`rak_interfaces`）

```
# ToolProposal.msg —— "想让机器人做一件工具菜单里的事"。这是提案，不是命令：
# 执行前必过确定性前置检查与护栏，任何一方可以拒绝。
std_msgs/Header header            # stamp=提议时刻；frame_id 对 arm.reach 必须是 base_link
string call_id                    # uuid，提案与结果对账
string source                     # operator | harness   （v1 只有这两个；s2 预留，Harness 落地后由 Harness 生成）
uint16 schema_version             # 工具 schema 表版本；不认识=拒绝
string name                       # 工具名，如 "arm.reach"
string args_json                  # 严格 JSON，按 schema 表逐键校验；未知键=拒绝
```

```
# ToolResult.msg —— 每个 call_id 恰好一条终态
std_msgs/Header header
string call_id
string name
string status                     # rejected | accepted | running | succeeded | failed | canceled | timeout
string message                    # 人读原因，绝不解析
string detail_json                # 机器读细节：轨迹 id、耗时、终态容差、IK 解、护栏触发原因……
```

- 话题：`/tool_proposals`（可靠 QoS，keep_last 10）、`/tool_results`（同）。
- **Schema 注册表**：`rak_tool_executor` 内 stdlib-only 的版本化表：`name → (args 校验器, 安全级, 默认超时, 可逆性)`。加新工具 = 加表项（additive，不需要动消息）；改已有工具的 args 语义 = 升 `schema_version`。这与 `rak_intent` 的"版本化文本契约"先例一致。
- 语料：工具事件以 `kind="tool_call"` / `kind="tool_result"` 行写入当前 episode（同一 corpus，`call_id` 对账），操作员标注工具复用同一通道写 `label`。

### 3.1 消费方式：agentic loop——工具即手，不是给外面开控制台

本设计不设 CLI 操作台。控制面是**机器人自己的循环**；跨进程 MCP 工具面是 ADR-0028
接受的第二个客户端（`src/rak_mcp`），它只消费同一条提案缝（`/tool_proposals` +
`/tool_results`），权限/护栏/审计仍只在 `rak_tool_executor` 里执行，
和 Claude Code 调 Read/Write/Bash 是同一种结构，只是"世界"从文件系统换成了硬件：

```
操作员（语音 ASR/开发台文本）
   │ 自然语言
   ▼
loop（rak_harness 最小环 = ADR-0010 design-only 目标的第一个垂直切片）
   ├─ 上下文 = 感知摘要 + /brain/decision（上下文，非命令）+ 工具历史 + 确认状态
   ├─ LLM function-calling：工具定义 = schema 注册表导出的 JSON Schema（同一份表）
   │   （DeepSeek，Anthropic 兼容端点，ANTHROPIC_API_KEY——ADR-0010 既定）
   ▼
ToolProposal(source=harness) ──► rak_tool_executor（前置检查 / 护栏 / 权限）
   ▼
ToolResult.detail_json ──► 回灌 loop 上下文 → 下一环继续（观察→动作→直到完成或被确认拦下）
```

与 Claude Code 的逐条对应（不是比喻，是等价物）：

| Claude Code | 本设计 |
|---|---|
| 工具表 + JSON Schema | schema 注册表（§3，单一来源：同一份 JSON Schema 喂 loop 的 function-calling 与执行器校验） |
| `allowedTools` / 危险命令前的确认提示 | 权限层：安全级 → 自动放行 / 每次确认 / 恒拒绝（§4 映射） |
| Bash 跑在 shell 子进程 | 工具跑在 `rak_tool_executor` 节点——跨进程是**安全边界**，不是开销 |
| 工具结果回灌上下文 | `ToolResult` 回灌 loop，形成观察-再动作 |
| 会话审计 | 语料事件行 + `call_id` 对账 |

**监督即权限（v1 的运行模式）**：`read`/`speak` 级自动放行；`motion_*` 级**每次人在环确认**
（loop 说"确认执行 arm.reach：tool0 到 (x,y,z)，低速？"，操作员语音/文本确认——恰如
Claude Code 在越权 Bash 前的提示）；`operator_only` 对 loop 恒拒绝。预批准类别/限速白名单
是后续按证据放开的策略位，不是 v1 决定。

语音通路与既有 `rak_intent`→`task_executor` 直通**并存不抢道**：简单导航意图仍走确定性
intent 直通；loop 负责多步、臂、工具、需要观察-再动作的任务。开发台文本是 v1 的等价输入
（同一个 loop 入口，不加工具权限差异）。

外部桥（rosbridge、ros-mcp-server 一类）**不是本架构的一部分**：不给外部客户端工具权限是
设计决定——通用 ROS 访问权会绕过菜单、护栏与审计。需要看图用 Foxglove/ros2 topic echo，
台架自便；生产栈不启动它们。

**loop 本体的实现形态见 D7**：官方 `anthropic` SDK 做传输层，agentic loop 用 Claude Agent
SDK；下表"逐条对应"里的权限层/审计都由 ROS 侧落地。P1 落地事实（2026-10-15）：

- SDK（0.2.167，内嵌 CLI 2.1.300）**原生运行在 aarch64**（Bun 单文件，无需 Node）；
- loop 跑在 `harness_node` 的**子进程**里（SDK 依赖树含 pydantic v2，与 ROS 系统 Python
  的 v1 隔离）；PYTHONPATH 只注入 `tools/harness-sdk` 覆盖层；
- `CLAUDE_CONFIG_DIR` 隔离到覆盖层目录——否则子进程会继承操作员 `~/.claude` 的 hooks
  （实测一个 SessionEnd hook 缺 node 会直接判会话 fatal）；
- 模型名经 `ANTHROPIC_CUSTOM_MODEL_OPTION` 透传（CLI 有 Anthropic 模型白名单，
  `unrecognized_model` 只是 warning，DeepSeek 的 `deepseek-chat` 可到端点）；
- 工具暴露：read/speak 恒暴露，motion_* 仅当 `agent_allow_motion_tools:=true`；
  执行权威与 `operator_confirmed` 判定始终在 `rak_tool_executor`（本次实飞用参数自动确认，
  逐次语音确认是下一步）。

## 4. 工具菜单 v1

安全级：`read`（只读）｜`speak`（发声）｜`motion_free`（自由空间运动）｜`motion_on_object`（接触物体）｜`operator_only`（仅操作员，agent 永不可用）。

| name | args（严格键） | 执行者 | 安全级 | 前置检查 | 超时/中止 | 可逆性 |
|---|---|---|---|---|---|---|
| `query.joints` | — | `/joint_states` 读取 | read | — | 即时 | n/a |
| `query.telemetry` | — | `BaseTelemetry` 读取 | read | — | 即时 | n/a |
| `say` | `text`（≤200 字符，非空） | `/speech/say` | speak | TTS 在线 | 队列即返 | n/a |
| `task.text` | `text`（非空） | `ExecuteTask`（rak_task_executor） | motion_free | 无臂运动进行；executor 空闲 | action cancel 路径 | PAUSE/RESUME/CANCEL |
| `task.goto` | `waypoint`（别名）**或** `pose`（PoseStamped，map 系）二选一 | 同上 | motion_free | 同上 | 同上 | 同上 |
| `task.pause/resume/cancel` | `task_id` | `TaskControl` | read→motion | 任务存在 | 即时受理 | 是 |
| `arm.reach` | `pose`（PoseStamped，**base_link 系**）、`speed_scale`（0<v≤0.25）、`timeout_s`（默认 15） | `/solve_ik` → `arm_controller` | motion_free | `allow_motion` 已开、无急停、臂护栏无人（§6）、无并发底盘任务 | `cancel`→停发并保持当前位置；超时→cancel+`timeout` | 是（可回 Home） |
| `arm.home` | `speed_scale` | 同上（Home=`[0, 3.9270, 4.7124, 0, 0, 0]` rad，契约 §1） | motion_free | 同上 | 同上 | 是 |
| `gripper.set` | `position`（0…1） | `gripper_controller` 话题 | motion_on_object | **硬件门**：无物理通道期间恒 `rejected`（原因明示） | n/a（无反馈） | 语义上是 |
| `scene.scan` | `degrees` | （P4：WBC 底盘自转） | motion_free | **推迟**：底盘单写者冲突未解 | — | 是 |
| `query.chassis_state` | — | `/joint_states` 四轮 + `mecanum_drive_controller` odometry | read | — | 即时 | n/a |
| `chassis.drive` | `vx_mps`/`vy_mps`（≤0.30）、`wz_rps`（≤1.20）、`duration_s`（0.05…3.0，必填） | `/mecanum_drive_controller/reference` @20 Hz | motion_free | `allow_motion` 已开；**Nav2 不在场**（互锁，ADR-0028） | 到点自动置零；让出时置零 | 是 |
| `chassis.stop` | — | 同上，只写零 | read（绕过 `allow_motion`，逐次审计） | — | 即时 | 是 |

> 菜单的**权威清单**是 `rak_tool_executor/tool_registry.py`（`tools/list` 与 MCP 面同源）；
> 本表是人读视图，改工具请两边一起对。

**显式禁止**（提案即 `rejected` 并审计）：`estop`、`motion_enable`（两者都是 operator_only 硬件门，不进菜单）、任何速度/关节直写、任何标定参数写（`wheel_sign` 等）。

**权限映射（loop 权限层，§3.1 的执行侧）**：`read`/`speak` → loop 自动放行；
`motion_free`/`motion_on_object` → v1 **每次人在环确认**（操作员确认后才成为可执行提案）；
`operator_only` → 对 loop 恒拒绝（人工直接注入也必须硬件门开着）。确认状态与确认人记入
`ToolResult.detail_json`，与 `call_id` 一起进语料——这是将来"自主放权"要吃的证据。

## 5. `arm.reach` 链路（P2 的核心，逐段确定性）

```
ToolProposal(arm.reach)
  → 前置检查（§4 表）：allow_motion? 急停? 臂护栏? 并发任务?  ——任一不过 → rejected+审计
  → TF: pose 从 frame_id 解析到 base_link（超时/外推失败 → rejected）
  → /solve_ik（GetPositionIK, group=arm, 真臂 URDF+契约关节名, DLS 迭代）
      失败/NaN/越限（0…4.71239 rad） → rejected（detail 带失败码）
  → 运动学 sanity：与当前关节位姿的 max|Δq| 限幅（防止"瞬移"目标），
    速度按 URDF 限位 × speed_scale（≤0.25）恒速直线插值，加速度限幅（保守值，P2 实测回填）
  → FollowJointTrajectory（arm_controller）
  → 终态：JTC succeeded 且 |误差|≤ 0.05 rad（契约口径 goal 容差）→ succeeded
    （语义：**指令按回读达标**；回读=指令回声，非物理确认——写进 detail_json）
  超时/外部 cancel → JTC cancel → 目标=current（舵机保位）→ timeout/canceled
  执行中护栏触发（§6）→ 立即 JTC cancel → canceled（detail 记录触发实体 id/距离）
```

- 全程无插值黑盒：P2 只有"直线关节插值 + 恒速"，轨迹在 RViz 可预演、在纸上可算。
- 一次一个 goal：工具执行器内部队列长度 1，新提案在执行期按菜单规则拒绝（`arm.*` 拒绝，`task.*` 拒绝，`read/speak` 放行）。

## 6. 安全与权威

| 主体 | 提案工具 | 直接执行 | 否决/覆盖 |
|---|---|---|---|
| 硬件急停 | — | 轮驱停 | 一切（**但不停舵机**——缺口见 §9） |
| `guard()`（场景几何否决） | — | stop | 一切，含臂提案（人进入底盘 stop 区时臂提案同样拒绝） |
| 臂工作空间护栏（本设计新增，`guard()` 的臂版精神） | — | cancel+hold | 执行中的臂运动：感知中任一实体进入以 base 为圆心 r（默认 0.8 m）且高度进入臂包络 → 立即取消。确定性、无模型参与，与 `guard()` 同一感知源 |
| 操作员 | 全菜单 | 全菜单（含 operator_only，经直接注入通道，仅台架/commissioning） | 可 override 一切提案决定；运动类=确认人 |
| Harness loop（P1 起，人在环监督） | 全菜单（除 operator_only） | 无 | 只能**收窄**（更慢、更小范围、拒绝），不能放宽速度上限/护栏/门；motion 级每次确认 |
| S1 Laya | **无**（只有 7 类决策） | 无 | 通过 gate 层级间接约束提案执行档位 |
| WBC（P4 后） | — | 唯一常规速度写者 | 底盘+臂并发的仲裁者 |

不变式：工具执行器是 `arm_controller` 的**唯一**客户端（P2–P3）；`rak_commissioning` 的示教链路在工具执行器上线后迁移到同一契约（避免第二个执行客户端）。

## 7. 与 Laya 和训练的关系（"最后才是训练环节"的具体化）

1. **语料先行**：P0 落地的操作员标注工具把 `label` 槽（现 2317 行全空）流转起来；工具事件行让"决策-动作-结果"闭环全部入 corpus。
2. **flags 增值不动格式**：`arm_moving` / `gripper_holding` / `task_active` 等 flags 值让 `state_text` 带上操作上下文——Laya 的微调输入因此包含臂态，而历史语料保持合法（旧行为=旧 flags 子集）。
3. **微调在仓外**：421M bf16，LoRA/PEFT（<8 GB 显存的外部 CUDA 机；本机只做推理/导出）。上游 `rai_finetune` 可用性是开放项（见 `loop.py` 注释），大概率需自写最小监督循环（choice 头 CE + act 头目标）。
4. **回环不可跳过**：微调后 → ONNX 导出（`docs/how-to/laya_tensorrt_runbook.md` 三个坑）→ **Jetson 重建 .plan**（绑定本机）→ 重跑 §Acceptance + `test_tensorrt_decider.py` 钉值（marker 布局、空场景 token 数）→ `gate()` 阈值按标注语料重拟合。
5. **标签集扩展是 v2 的事**：若主动智能 v2 需要臂动作标签进入 S1，那是新 checkpoint 版本 + 新引擎 profile（marker 数变）+ `ActionKind` breaking change——必须单独 ADR，且先有 v1 微调的增益证据再决定值不值。

## 8. 分期计划（每期有验收，不跨期冒进）

**P0 — 契约与执行器骨架（无 loop、无运动）**
新包 `src/rak_tool_executor`（见 §10）：schema 注册表（每表项含 JSON Schema）、
`/tool_proposals`→`/tool_results` 全路径、权限判定（自动/确认/恒拒绝）、rejected 审计、
语料事件行、操作员标注工具。臂菜单项恒 rejected-unimplemented。
验收：`colcon build` + `check_workspace.py` 绿（EXPECTED 增补）；直接注入通道
（`ros2 topic pub`，纯调试用）发起 `read/speak/motion` 各级提案，权限判定与审计 100% 正确、
语料有对账行；两提案并发只有一个 accepted。

**P1 — loop 垂直切片（无运动）**
最小 `src/rak_harness`（§10：这是 ADR-0010 design-only 目标的**第一个实现切片**，
实施裁定与本设计的 ADR 化一并处理）：LLM function-calling 环（工具定义=schema 表导出的
JSON Schema）、`ToolResult` 回灌、权限确认流（motion 级人在环确认）、语音/开发台两个等价输入。
验收：操作员**用自然语言**让 loop 完成"查关节状态 → 说一句话"多步任务（观察-再动作）；
motion 级提案必然触发确认、确认状态入 `detail_json` 与语料；LLM 断供时 loop 静默降级
（不再产生新提案），`guard()`/S1 安全路径不受影响；无任何硬件被驱动。

**P2 — 臂：单点 IK 到位（经 loop + 确认）**
真臂参数化起 `/solve_ik`；`arm.reach`/`arm.home` 全链路 + 护栏 + 取消/超时路径；速度 25% 上限。
验收：先 `allow_motion:=false` 全链验证（goal 被抑制、提示可读）；再实机单点到位×6 关节、护栏用实体逼近实测 cancel、超时路径实测；detail_json 语义（指令回读≠物理确认）经操作员确认。**软件侧先用六轴 fixture 跑通逻辑（fixture 永不上实机臂）**。

**P3 — 抓取闭环（准静态）**
`GraspCandidates` → `SelectGrasp` → TF 到 base_link → `grasp.reach`（GraspNet +X 接近轴约定 → tool0 的固定偏置，**需实测标定**）→ `gripper.set`（硬件门通过后）；评估引入 MoveIt 碰撞规划。
验收：sim fixture + 离线快照回放全绿后才上实机；实机低速、单候选、准静态；无接触力反馈的事实全程明示。

**P4 — 全身（WBC 闭合后）**
`scene.scan`、底盘-臂并发提案由 WBC 仲裁；`publish_arm` 开门走 ADR-0011 待闭合项流程。

**P5 — 训练**（§7：标注→外部 LoRA→导出回环→阈值重拟合）。

### Bench 记录（2026-10-15，全链条实机）

- **语音→思考→物理→语音全通**：`/speech/transcript` → loop（思维链 trace 上
  `/harness/trace`）→ `query.joint_states`（真实读数，LLM 复述正确）→
  `arm.home`（真机臂动，`Goal successfully reached`，12s 到 [0,225,270,0,0,0]°）→
  `say`（kokoro 出声）。persona=小锐，prompt=2026-10-15.3。
- **决定性发现**：SDK 必须 `permission_mode="bypassPermissions"`——ark-code-latest 在
  acceptEdits 下会脑补“等弹窗放行”而拒绝调用任何工具（含 query）；执行器权威不变。
- **模型边界**：ark-code-latest（账号唯一 provision 款）工具选择弱、显式点名工具才
  稳定调用；查询结果复述准确。换更强模型是运营项，不是代码项。
- **ASR**：默认引擎切流式 Zipformer 14M（`engine:=zipformer`），wav 实测识别完整中文
  句子；Vosk 回退保留。
- **TTS**：kokoro 现役；CosyVoice3-0.5B 代码全通（`backend:=cosyvoice` + Say.srv
  instruct + 人格调色板），实机推理待 GPU 堆愈合（重启）与 headless 显存。
- **环境**：torch 2.12.0+cu132 与 onnxruntime-gpu 1.28（TensorRT EP）装用户 site；
  numpy 钉 1.26.4；CosyVoice 的 torchaudio/pyworld/pyarrow/modelscope 依赖用
  tools/cosyvoice 内垫片与懒加载解决（pydantic v1 未被污染，ROS 侧全绿）。

## 9. 开放问题与硬件门

1. **夹爪无物理通道**：P2 实测确认 GRIP 命令的物理效果；不通则 `gripper.set` 保持恒 rejected。
2. **急停不停臂**：向固件提需求（臂运动许可与急停联动）或接受上位机限速+护栏的现状——需要一次明确裁定。
3. **无编码器**：无接触/卡死检测；抓取准静态是纪律不是选项。
4. **最小 loop = ADR-0010 的第一个实现切片**：P1 创建 `src/rak_harness` 意味着把"accepted
   but design-only"的 Harness 真正建起来——本设计的 ADR 化应与 ADR-0010 的实现裁定一并处理，
   且提案缝不得因此出现第二个。
5. **IK 服务单例**：`/solve_ik` 现由 `pinocchio_ik.launch.py` 启动（默认 fixture 参数）；P2 需要一份**真臂参数的启动入口**（launch 参数化），并与 MoveIt 侧共存策略（谁拥有 robot_description）需要在 P2 开工时定稿。
6. **IMU 无应答**（2026-10-14 实测，`WHO_AM_I=0x7f`）：不影响臂 P2，阻塞 whole-body 标定。
7. **臂执行期底盘策略**：P2–P3 拒绝并发导航（D5）；是否允许"底盘移动+臂折叠 Home 位"的复合状态，留给 P4/WBC 裁定。
8. **loop 的 LLM 供应与延迟**：单环 12–30 s（与臂动作同量级，可接受），但断网/供应商故障时
   loop 必须静默降级——不再产生新提案，已确认的运动走完，`guard()`/S1 路径完全独立不受影响
   （P1 验收项）。密钥只在环境变量（ANTHROPIC_API_KEY），任何配置文件不得落键。
9. **不接通用远程控制桥**：不接 rosbridge 之类给外部客户端的无鉴权控制桥；台架需要看图
   用 Foxglove 或 `ros2 topic echo`。**跨进程 MCP 工具面已由 ADR-0028 接受**（stdio、
    本地进程、复用提案缝与同一套护栏），其余远程遥操作需求仍需单独 ADR 并先定鉴权。
10. **DeepSeek Anthropic 兼容端点的 tool_use 支持需先探针**（P1 第一件事）：最小验证 = 一条
    工具定义 + 一次 tool_use 往返 + `tool_result` 回灌；不通过则 loop 传输层切 SiliconFlow
    （OpenAI 兼容格式，消息形状相同，loop 本体不动——D7 的分层就是为这个）。

## 10. 结构影响（ADR-0013 结构法）

- 新增 `src/rak_tool_executor`（ament_python；依赖仅 `rclpy`、`rak_interfaces`、标准消息——**不得**依赖 `rak_agent`/`rak_chassis`/`rak_commissioning`，方向向下）。单入口：提案执行器节点（schema 表、权限层、执行、审计）。
- P1 新增 `src/rak_harness` 最小环（D7：官方 `anthropic` SDK 做传输层 + Claude Agent SDK
  loop，跑在 harness_node 的子进程里隔离其 pydantic v2 依赖树）。这是 ADR-0010 目标的
  第一个实现切片，结构增补随其 ADR 裁定一起走。
- SDK 落在 gitignored 的 `tools/harness-sdk/`（`pip3 install --target`，与 `tokenizers`
  同级的用户侧先例；机器本地，不进 colcon 图）。`rak_harness` import 失败时只禁音 loop
  （harness_node 回落 ADR-0010 对话引擎），提案缝照常。
- `rak_interfaces` 追加 `ToolProposal.msg` / `ToolResult.msg`（additive，非 breaking）。
- `scripts/check_workspace.py` 的 EXPECTED 集合在 P0/P1 同步增补（结构法：改结构，不改断言）。
- `rak_commissioning` 保留现有示教链路至 P2 迁移到同一提案契约（避免第二个执行客户端）。
- `rak_agent` 零改动（全程）；其 S2 若要产提案，须作为 loop 的上下文经 Harness（ADR-0010）——另行裁定。
