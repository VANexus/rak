# ADR-0010: Jev + Harness 任务仲裁，以及「呼吸感」预留表达接口

- **状态**: 已接受（2026-10-07 用户确认：Harness 走标准 ROS 2 包 + 独立节点）
- **日期**: 2026-10-07
- **相关**: `brain/rak_ai/task_intent.py` · `brain/rak_ai/system1.py` · `brain/rak_ai/system2.py` · `brain/rak_ai/node.py` · `brain/rak_ai/tensorrt_decider.py` · `src/rak_task_executor/rak_task_executor/node.py` · `src/rak_interfaces/msg/Decision.msg` · `src/rak_speech/rak_speech/asr_node.py` · `docs/explanation/architecture.md` §2/§4/§6/§7 · `docs/reference/hardware_acceptance.md` · ADR-0007 · ADR-0008

---

## 背景

`README.md` 把工程目标写成一句话：**Jev 快速选择，云端 LLM + Harness 理解任务**。
但到今天为止，这句话的后半句在代码里没有对应物 —— `Harness` 只出现在散文中
（`docs/explanation/architecture.md:107,114,218,230`），没有类、没有节点、没有话题。
本文要回答的是：**在既有冻结契约与单一生命周期所有者的前提下，Harness 该长成什么样，
以及它与 Jev（本机 System 1）如何共享仲裁。**

### 已核实事实（2026-10-07 直接查代码）

- **Jev 已就绪**：`brain/rak_ai/tensorrt_decider.py` 的 `TensorRtDecider.decide()` 在
  `rak_agent` 进程内直接调用（`brain/rak_ai/node.py:107` `_make_decider`，`decider:=tensorrt`），
  单次 ~58 ms。引擎与 ONNX 资产见 `docs/how-to/laya_tensorrt_runbook.md`。它离域、未标定，
  在真实场景里**恒为 LOW 置信度**（p≈0.22），因此**按设计每 tick 都升级**。
- **安全权威只有 `guard()`**：`brain/rak_ai/system1.py` 里 `guard()`（任一实体 `0<z<0.6`
  且 `|x|<0.5` 即否决）是唯一可授权停车的组件；`gate()`（`HIGH_P=0.70`/`HIGH_MARGIN=0.30`/
  `MEDIUM_P=0.40`）只选升级档；`resolve()` 合成 `(Decision, Tier, Action|None)`。
- **`brain/rak_ai/types.py` 冻结**：`state_text` 逐字进 `brain/data/episodes/*.jsonl`。
  新协议必须另行版本化（`task_intent.py` 开头已有此先例）。
- **唯一已有的云端缝**：`brain/rak_ai/task_intent.py:44-55` 的
  `SlowTaskPlanner.propose(text, *, task_id, revision, allowed_waypoints) -> WaypointSelection`，
  由 `RaiWaypointPlanner` 实现，被 `src/rak_task_executor/rak_task_executor/node.py:550-593`
  的 `_waypoint_from_text()` 调用，`slow_planner ∈ {local, rai}`。
  它是**一次性、无状态、只回白名单别名**的同步函数。
- **`rak_task_executor` 是唯一导航任务生命周期所有者**：`/task/execute`(Action)、
  `/task/control`(Service)、`/task/status`(Topic)；`ActiveTask` 有 `task_id`(uuid4)、
  `revision`(uint64，每次控制自增)、`request_id`(256 条去重)。
- **`/brain/decision` 全仓无消费者**：`src/rak_interfaces/msg/Decision.msg` 有
  `source∈s1|s2|guard|fallback`、`p`、`margin`、`latency_ms`、`t`、`tier`，
  但只有 `brain/rak_ai/node.py:42` 的发布侧 —— S1 决策今天到不了执行器。
- **`src/rak_speech`**：Vosk ASR 发布 `/speech/transcript` 与 `/speech/intent`(JSON)。
  **全仓无 TTS**；`brain/config.toml [tts]` 指向 ElevenLabs、`voice=""`、未被使用。
- **`command_id` / `expires_at` 不存在于代码**，只在 `docs/explanation/architecture.md:136-139` 的提案表里。
- **已装 / 未装**：`vosk` 已装；`anthropic` / `openai` / `rai` / `langgraph` **均未装**。
  运行时是 ROS 系统 Python 3.12。

### 硬约束（本文必须逐条对上）

1. `types.py` 冻结；新协议另行版本化。
2. `guard()` 唯一权威；`gate()` 只选档；System 2 **只能收窄，不能放宽**。
3. 云端 LLM 必须 fire-and-forget / 异步；S2 实测 12–30 s，**不得进入实时控制环**；
   控制路径不得同步等云端。
4. System 1 保持**进程内直接函数调用**；快路径无编排框架、无跨进程。
5. `rak_task_executor` 是唯一生命周期所有者；不得加第二个 Nav2 或竞争性 `/cmd_vel` 写者。
6. LLM key 只来自环境变量；`brain/config.toml` **永不**存 key。
7. 硬件锁定（`docs/reference/hardware_acceptance.md`），本轮无真实底盘/机械臂动作。
8. `docs/explanation/architecture.md` §2 规则 7：未实现能力**先定义设计与验收**，不为「看起来完整」创建空包。
9. `docs/explanation/architecture.md` §4：**只有独立部署或资源隔离确有必要时才新增进程**。

### 问题

Jev 已经能在 58 ms 内对每一帧给出选择，但：

- 它**离域不可信**（恒 LOW），所以它的价值不是"直接执行"，而是"作为事件源参与更高层判断"；
- 它**到不了任务层**（`/brain/decision` 无消费者），所以任务执行器看不到"眼前发生了什么"；
- 任务层目前只有**一次性**的地点选择（`SlowTaskPlanner`），没有持续上下文、没有失败恢复、
  没有事件摘要 —— 即"有智能，没有呼吸"。

**Jev 与任务执行器之间该有一条什么样的仲裁缝，才能既让 Jev 影响任务、又不给任务状态机
加第二个权威？"呼吸感"又该落在哪里，才能不被硬件门和实时环两头卡死？**

---

## 决策

**新增标准 ROS 2 包 `src/rak_harness`（含独立节点 `harness_node`），作为 Jev 与
`rak_task_executor` 之间唯一的、传递式的仲裁缝；云端 LLM 走新的独立 `anthropic` 客户端；
「呼吸感」拆成"软件可表达（本轮可做）"与"硬件可表达（本轮只留接口 + 门控 stub）"两层。**

```text
perception ─▶ rak_agent: Jev(S1, 进程内) ─▶ /brain/decision ─┐（只读上下文，不赋权）
                                                            │
/speech/intent ─────────────────────────────────────────────┤
/task/status  ──────────────────────────────────────────────┤
                                                            ▼
                                        ┌───────────────────────────────┐
   /harness/request  (executor → harness)│  src/rak_harness              │
   /harness/proposal (harness → executor)│  harness_node                 │
   /harness/event    (harness → UI)      │  ├ cloud_client (anthropic)   │
                                        │  │   → api.deepseek.com/anthropic
                                        │  └ expression_contract (预留) │
                                        └───────────────────────────────┘
                                                     │ 提案（结构化，无速度/关节）
                                                     ▼
                                        rak_task_executor（唯一生命周期所有者）
                                                     │
                                                     ▼  Nav2 / MoveIt / 表达（门控）
```

### D1. Harness = 标准 ROS 2 包 `src/rak_harness` + 节点 `harness_node`

- **包**：`src/rak_harness`（`ament_python`，与 `brain/`、`src/rak_speech` 同构）。
- **节点**：`harness_node`（console entry point），由 `rak_bringup` 与任务栈一并启动，
  与 `agent_node` 并列，**不**嵌入 `rak_agent`。
- **包内文件**（实现时创建，本轮只定义）：
  - `rak_harness/node.py` — 节点与订阅/发布接线
  - `rak_harness/contract.py` — 版本化契约（`schema_version=1`），**独立于冻结的 `types.py`**
  - `rak_harness/cloud_client.py` — 新 `anthropic` 客户端（见 D3）
  - `rak_harness/expression.py` — 表达层契约与 no-op（见 D4）
- **依赖**：`rak_interfaces`（新消息）、`rclpy`、`std_msgs`，以及**可选**的 `anthropic`。
  `anthropic` 用**延迟导入 + 缺失降级**（照 `brain/rak_ai/system2.py` 的 `RaiConsultant`
  与 `brain/rak_ai/node.py` 的 `_rai()` 写法），未装时 `harness_node` 以明确日志降级为
  "只发 NO_OP"的 stub。运行时仍是 ROS 系统 Python，**不加 venv、不切解释器**。

**为什么是包而不是 `brain/rak_ai` 里的模块**（本条为 2026-10-07 用户裁定，覆盖设计稿初议）：
`docs/explanation/architecture.md` §2 规则 2 明确"包是构建、依赖、部署和测试边界"。`anthropic` SDK 及其
HTTP/重试栈是一个**与 ROS 认知原型不同的依赖与部署边界**，且需要一个**独立崩溃域**
（SDK 的未捕获异常/内存不得污染 58 ms 的 S1 进程）。这同时满足 §4 的"资源隔离确有必要时新增进程"。
规则 7（"不为看起来完整建空包"）不被违反：本文先定义设计与验收，**实现时才创建包**，
且包内每一文件都有明确职责，不是占位。

### D2. 仲裁缝：`/brain/decision` 的唯一新消费者是 Harness；执行器不直连 S1

这是本 ADR 最关键的一条，它明确回答"Jev 的决策今天到不了执行器"。

- **`/brain/decision` 的消费者 = `harness_node`**，且**仅作事件上下文**（best-effort、
  keep-last-N），用于事件摘要、澄清触发与 `rationale` 生成。Harness **不得**据此直接产生运动。
  `Decision.msg` **不改字段**（新增字段会动到既有消息契约，且本决策不要求）。
- **`rak_task_executor` 不订阅 `/brain/decision`**。理由：
  1. `guard()` 的实时否决属于**命令出口**（`docs/explanation/architecture.md` §4「安全许可应落实到命令出口」），
     不属于任务状态机；执行器不写 `/cmd_vel`，让它消费 stop 只会制造"有安全"的错觉。
  2. 执行器已有 `/task/control` 与超时/失败进入 `SAFETY_HOLD` 的**唯一**输入面；
     再加一个能改任务状态的输入源 = 第二套仲裁。
  3. S1 离域、不可信，不能作为任务级权威。
- **Jev 触达执行器的唯一路径**是传递式的、非权威的：
  `Jev → /brain/decision → Harness（上下文）→ /harness/proposal → 执行器（校验后接受）`。

**新增接口（全部 `schema_version=1`，落在 `src/rak_interfaces`）**：

| 名称 | 方向 | 类型 | 用途 |
|---|---|---|---|
| `/harness/request` | executor → harness | Topic `HarnessRequest` | 规划请求：`task_id`、`revision`、`request_id`、`text`、`allowed_waypoints[]`、`deadline`、`context_flags[]` |
| `/harness/proposal` | harness → executor | Topic `HarnessProposal` | 结构化提案：`task_id`、`revision`、`request_id`、`kind`、`waypoint`、`rationale`、`expires_at` |
| `/harness/event` | harness → UI/console | Topic `HarnessEvent` | 人可读确认/进度/恢复事件（呼吸感软件侧，见 D4） |
| `/task/status` | executor → harness | 既有 Topic | Harness 的事件来源（只读订阅） |
| `/speech/intent` | ASR → harness | 既有 Topic | 后续对话入口（只读订阅） |
| `/brain/decision` | agent → harness | 既有 Topic | 事件上下文（只读订阅） |

`HarnessProposal.kind` 枚举（权限**刻意收窄**）：

```text
KIND_NO_OP=0                 # 不行动（预算耗尽/超时/无把握）
KIND_SELECT_WAYPOINT=1       # 只回白名单别名
KIND_REVISE_PLAN=2           # 计划修订，仍是结构化目标，不是速度/关节
KIND_REQUEST_CLARIFICATION=3 # 请求澄清，交给人
```

**执行器接受提案的全部条件**（缺一不可，任一不满足即丢弃）：
`task_id == active.task_id`、`revision == active.revision`、`now <= expires_at`、
`kind ∈ 执行器允许集`、`waypoint ∈ self._waypoints` 白名单。
这复用现有 `task_id`/`revision` 陈旧拒绝（`node.py:589-590` 已有先例）与
`resolve_waypoint` 校验（`task_intent.py:100-105`）。

`slow_planner` 参数扩展为 `local | rai | harness`：`local|rai` 走既有进程内
`SlowTaskPlanner` **不变**；`harness` 走"发布 `/harness/request` → 在既有 `PLANNING`
状态内以 `slow_planner_timeout_sec` 为界等待匹配的 `/harness/proposal`"。
**注意区分**：等待发生在**任务规划阶段（非运动环）**，且必然可超时进入 `FAILED`/`ask_operator`；
云调用本身在 Harness 内是 fire-and-forget，**执行器从不直接等云端**。

### D3. Harness ↔ 云端 LLM：新的独立 `anthropic` 客户端

**不复用、不扩展 RAI/SiliconFlow 路径**（用户已定）。新文件 `rak_harness/cloud_client.py`。

- **SDK / 端点**：`from anthropic import Anthropic`；
  `base_url="https://api.deepseek.com/anthropic"`（Anthropic 兼容端点，**显式传参**，
  不依赖 `ANTHROPIC_BASE_URL` 环境变量，避免隐式漂移）。
- **模型名**：配置值，默认占位 `deepseek-chat`，标注 `TODO: 实测替换`
  （须按 DeepSeek 官方文档核对该端点可用模型 id，不在 ADR 里臆断）。
- **Key**：新环境变量 **`ANTHROPIC_API_KEY`**（SDK 原生默认名），显式
  `api_key=os.environ.get("ANTHROPIC_API_KEY")`。**不复用 `OPENAI_API_KEY`** ——
  那是 RAI/SiliconFlow 的 key，两个 provider 共用一个变量是 key 混淆事故源。
  `brain/config.toml` 只放非密钥配置（`base_url`、模型名、超时、预算），**永不出现 key**。
- **超时 / 重试**：`timeout=20.0`（连接+读），SDK `max_retries=1`，仅对连接错误/5xx/429
  做带抖动退避；**`expires_at` 已过的请求不重试**。Harness 层另有单请求总截止时间与每任务调用预算。
- **结构化输出**：用 Anthropic **tool use** 强制
  `tool_choice={"type":"tool","name":"propose_plan"}`，工具 `input_schema` 直接镜像
  `HarnessProposal` 的字段。**schema 里没有速度/关节字段**；`waypoint` 用 `enum`
  （由 `allowed_waypoints` 动态构造）—— 白名单在 API 层先收窄一次，返回后再
  `resolve_waypoint` 收窄一次（双保险）。`schema_version` 不匹配即拒绝。
- **权限**：客户端不持有 `guard()`/`gate()` 引用，输出**恒为 advisory**；
  Harness 输出**永不**是 `cmd_vel`/关节命令（架构 §4 明令）。
- **异步形态**：**单工作线程 + 有界队列（容量 1，丢弃最旧）**，用同步 `Anthropic` 客户端。
  理由：仓库既有并发约定就是线程（`brain/rak_ai/system2.py`、`brain/rak_ai/node.py:_escalate`），
  不引入 asyncio/rclpy 集成复杂度；队列容量 1 天然"合并重复事件"
  （架构 §4「合并重复事件，不是每帧调用云端」）。
- **陈旧拒绝**：请求带 `task_id`+`revision`+`expires_at`；Harness 发布前先丢弃已过期或
  revision 已被更新的结果；执行器再按 D2 条件二次拒绝。
- **预算 / 失败恢复**：每任务云调用上限（配置，默认小值，如 3）+ 全局速率限制；
  预算耗尽/超时/异常 → 发 `KIND_NO_OP` 或 `KIND_REQUEST_CLARIFICATION`，
  **绝不伪造成成功**（对齐 `docs/explanation/architecture.md`「超时进入失败或安全阻断，不伪报」）。
- **缺失依赖**：未装 `anthropic` 时 `harness_node` 以明确日志降级为"只发 NO_OP"的 stub。

### D4. 「呼吸感」表达接口：预留 + stub，本轮无真实动作

拆成两层，明确哪层本轮能兑现：

**第一层：软件可表达（本轮可做，无运动、无 TTS）** —— 通过 `/harness/event` + 既有文字控制台：

- **及时确认**：`/task/execute` 被接受后，Harness 在配置时限内发 `KIND_ACK`，带
  `ack_latency_ms`。验收指标是"接受→可见确认"的 p95，而非"模型多想了一会儿"。
- **自然停顿**：事件**合并去重**，不刷屏；状态未变不发重复事件
  （照 `brain/rak_ai/node.py:_record` 的"一个事实不是十次每秒"写法）。
- **连贯动作**：本轮 N/A（无硬件），接口保留。
- **可解释恢复**：`KIND_RECOVERY` 必带 `reason`（人可读），禁止"模型猜测完成了"（架构 §4）。

**第二层：硬件可表达（本轮只定义接口，门全关，stub 恒 no-op）**：

- `rak_harness/expression.py`（版本化）：
  - `ExpressionIntent`：`schema_version`、`kind ∈ {ACKNOWLEDGE, THINKING, PAUSE,
    ORIENT_ATTENTION, IDLE_POSTURE, SPEAK}`、`ttl_sec`、`text`（仅 SPEAK）、`reason`、
    `requires_idle_resources=True`。
  - `ExpressionGate.allowed(intent) -> bool`：**只有**同时满足才为真 ——
    (a) 对应执行器的硬件验收门已过（查 `docs/reference/hardware_acceptance.md` 状态）；
    (b) 资源空闲（无活动导航/抓取控制流持有该资源）；
    (c) 有效许可（如底盘未 ARM、无人工接管冲突）。
  - `ExpressionSink` Protocol + `NoOpExpressionSink`：**本轮唯一实现恒 no-op**，
    `ExpressionGate` 恒返回 `False`（硬件门未过）。
- ROS 面：`/expression/intent`(Topic, reserved) + `/expression/state`(Topic, reserved)。
  本轮**不启动**真实表达节点，只留契约与 no-op。
- **TTS 缺口**：全仓无 TTS；`brain/config.toml [tts]` 的 ElevenLabs 未接、`voice=""`。
  `SPEAK` 是预留缝，**TTS 引擎选型（离线 vs 云）列为待闭合项，本轮不做**，
  且不得把 `speak` 接到活动控制流上。

**硬性红线（架构 §4 逐字）**：待机姿态/朝向关注/语音节奏**只使用空闲资源与有效许可**；
**不叠加**到活动导航/抓取控制流；**不靠随机晃动掩盖系统迟延**。

---

## 理由

1. **D1 用标准 ROS 2 包**，直接满足 §2 规则 2 的"包 = 构建/依赖/部署/测试边界"：
   `anthropic` 是非 ROS 依赖，且需要一个独立崩溃域，这正是"资源隔离确有必要"。
   规则 7 不被违反 —— 本文先定义设计与验收，实现时才落包。
2. **D2 让 `/brain/decision` 有消费者但不赋权**：Jev 的影响是**传递式**的，
   `guard()` 唯一权威与"执行器唯一生命周期所有者"同时成立；且**不需要改 `Decision.msg`
   或 `types.py`**（冻结契约零改动）。
3. **D3 独立 anthropic 客户端**隔离两个 provider 的 key/端点/重试语义；
   tool-use + enum 把白名单收窄到 API 层；线程 + 容量 1 队列复用仓库既有并发模式。
4. **D4 分层**：把"呼吸感"中能无硬件兑现的部分（确认/停顿/可解释）与必须硬件门的部分
   （姿态/朝向/节奏）分开，避免"设计空转"，也避免表达泄漏进控制流。

---

## 后果

### 正面
- 在不违反冻结契约、单生命周期、单安全权威的前提下，**第一次定义了 Jev 与执行器的共享仲裁缝**。
- `guard()` 的语义在任务层有了**架构性（非实时）**表达路径：经 Harness 转成提案，而非新增安全权威。
- "呼吸感"有了**带门控的预留接口**，结构上无法泄漏进控制流。
- 新云端路径与 RAI 完全隔离；key 仅环境变量；缺依赖可降级。
- 诚实定界：无空包、stub 契约先行。

### 负面（代价，必须承认）
- **Harness 本轮只有接口**，"智能"仍是愿景，需实现 + 验收才可宣称。
- **新增一个包与一个进程**（`src/rak_harness` / `harness_node`）与一个系统 Python 依赖
  （`anthropic`），在 3.4 GB 板上是真实的内存/运维成本。
- **新增 3 份版本化契约**（`contract`、`expression`、新消息）与 **4 个话题**，
  维护面扩大，陈旧/过期/去重都需要专门测试。
- **话题式 request/proposal 比函数调用多很多活动部件**，正确性依赖
  `task_id`/`revision`/`expires_at` 三件套的测试覆盖。
- **"呼吸感"本轮几乎无用户可见收益**（无运动、无 TTS）。
- **云规划引入 12–30 s 延迟**；对**已知** waypoint，比现有 local 路径更慢。
- **两个 LLM provider 并存**（SiliconFlow/RAI 与 DeepSeek/anthropic），
  key、配置、可观测性负担上升。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 扩展既有 RAI/SiliconFlow 路径（`RaiWaypointPlanner` / `RaiConsultant`）承载 Harness | 用户已定"新独立 anthropic 客户端"；RAI 带自己的 agent loop/编排语义，且未安装；Harness 要的是纯 request/response + 版本化契约 |
| Harness 作为 `brain/rak_ai` 内的模块（不建包） | 用户 2026-10-07 裁定走标准 ROS 2 包；`anthropic` 是独立依赖与崩溃域边界，符合 §2 规则 2 |
| 在控制环/`resolve()` 内同步调云端 | S2 12–30 s 会卡死 58 ms 的 S1 tick；违反"控制路径不同步等云端" |
| Harness 作第二个 Nav2 / 第二个 `/cmd_vel` 写者 / 第二个生命周期所有者 | 违反执行器唯一所有者；Harness 只输出提案 |
| 把"呼吸感"叠进活动导航/抓取控制流，或随机晃动掩盖迟延 | 架构 §4 明文禁止；会制造安全与连贯性双重问题 |
| 扩展冻结的 `types.py` 或给 `Decision.msg` 加字段 | `types.py` 冻结（episode corpus 逐字存储）；新协议另行版本化（`task_intent.py` 先例） |
| 执行器直接订阅 `/brain/decision` 并把 S1 当权威 | S1 离域、不可信；`guard()` 权威属于命令出口；会给任务状态机加第二输入源 |
| Harness 走 Service / Action（而非 Topic） | Service 会让服务端回调阻塞 12–30 s 且耦合生命周期；Action 会造出竞争性 goal 生命周期与取消语义 |
| 新客户端复用 `OPENAI_API_KEY` | 两个 provider 共用一个变量，是 key 混淆事故源；改用 `ANTHROPIC_API_KEY` |
| 新客户端用 asyncio 事件循环 | 与仓库线程约定不符，rclpy+asyncio 集成风险高；本轮吞吐无此需求 |

---

## 待闭合项

1. **Harness 本体未实现**：本轮仅契约 + stub（`harness_node` 只发 NO_OP）。
2. **云端点/模型 id 未实测**：`https://api.deepseek.com/anthropic` 的可用模型名与
   tool-use 行为需实测；`deepseek-chat` 为占位（`TODO: 实测替换`）。
3. **`anthropic` SDK 未安装**：需在系统 Python 上确认可装、版本、与 ROS 共存。
4. **无实时安全路径**：S1/`guard` 的实时停车仍只在命令出口（未来），本 ADR 不提供；
   执行器 `SAFETY_HOLD` 仍只由 `/task/control`、超时、失败触发。
5. **`/brain/decision` → Harness 事件摘要的语义未验收**（哪些 tier/source 进摘要、去重窗口）。
6. **表达层硬件门全关**：无姿态/朝向/节奏；`ExpressionGate` 恒 false 只是占位。
7. **TTS 引擎未选型**：`SPEAK` 缝存在但无实现；`brain/config.toml [tts]` 仍不可用。
8. **`command_id` / `expires_at` 仍未进 `/task/execute`**：本 ADR 只在新契约里引入
   `request_id`/`expires_at`，未回填既有 Action。
9. **无端到端延迟 SLO**：harness 路径 p50/p95/p99 未测。
10. **多 provider 可观测性**：两个 key/provider 的日志与失败归因未设计。
11. **提案→执行的竞态测试**（迟到、过期、revision 变更、预算耗尽、Harness 重启）未写。

---

## 参考

- `brain/rak_ai/task_intent.py`（`SlowTaskPlanner` 缝）、`brain/rak_ai/system1.py`
  （`guard`/`gate`/`resolve`）、`brain/rak_ai/system2.py`（晚导入模式）、
  `brain/rak_ai/node.py`（S2 线程与陈旧拒绝先例）、`brain/rak_ai/tensorrt_decider.py`（Jev）
- `src/rak_task_executor/rak_task_executor/node.py`（唯一生命周期所有者）、
  `src/rak_interfaces/msg/Decision.msg`、`src/rak_interfaces/action/ExecuteTask.action`
- `src/rak_speech/rak_speech/asr_node.py`（ASR 现状；无 TTS）
- `docs/explanation/architecture.md` §2（包设计原则）、§4（认知侧 / 对话与表达侧）、§6（E 阶段）、§7（未解决问题）
- `docs/how-to/laya_tensorrt_runbook.md`（Jev 引擎资产与验收）、`docs/reference/hardware_acceptance.md`（硬件门）
- ADR-0007（统一 `ros2_control`）、ADR-0008（末端与感知套件）
- Anthropic Messages API tool use：<https://docs.anthropic.com/en/docs/build-with-claude/tool-use>
- DeepSeek Anthropic 兼容端点：<https://api.deepseek.com/anthropic>（模型 id 待实测）
