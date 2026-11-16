# Architecture Decision Records (ADR)

本目录记录**已经做出的架构决策**及其理由。与 `docs/explanation/architecture.md` 的区别：

| 文档 | 回答的问题 | 时间性 |
|---|---|---|
| `docs/explanation/architecture.md` | 系统现在**长什么样** | 描述当前状态 |
| `docs/status/project_status.md` | 现在**做到哪了** | 描述当前进度 |
| `docs/reference/hardware_acceptance.md` | 上车前**必须过哪些门** | 描述验收门槛 |
| `docs/adr/*`（本目录） | **为什么**是这样，以及否决了什么 | 不可变，只追加/取代 |

## 格式

每篇 ADR 固定骨架：

```markdown
# ADR-NNNN: <一句话决策>

- 状态: 提议中 | 已接受 | 已被 ADR-XXXX 取代 | 已废弃
- 日期: YYYY-MM-DD
- 相关: <代码/文档路径>

## 背景        —— 当时面对什么问题，有哪些硬约束
## 决策        —— 选了什么（祈使句，可执行）
## 理由        —— 为什么是它
## 后果        —— 正面 / 负面（代价必须写）
## 被否决的方案 —— 否决了什么，为什么（防止以后重复讨论）
## 待闭合项    —— 这个决策还没兑现的部分
```

## 硬性约定

1. **ADR 记录决策，不记录进度。** 进度写 `project_status.md`，任务写 `TODO.md`。
2. **状态一旦是"已接受"就不改写正文。** 要改就新写一篇，把旧篇标成"已被 ADR-XXXX 取代"。
3. **"后果 → 负面"必须非空。** 没有代价的决策通常意味着没想清楚。
4. **"被否决的方案"必须写清否决理由**，否则同一个问题会被反复提出。
5. **每条结论标注来源**：`CLAUDE.md` / 协议文档 / 实测 / 占位值（`TODO: 实测替换`）。
   本仓库有"每个文件要说明为什么存在"的工程约定（见 spec 4.3），ADR 同样适用。

## 索引

| 编号 | 决策 | 状态 |
|---|---|---|
| [0001](0001-mecanum-kinematics-on-mcu.md) | 麦轮逆运动学放在 F407，上位机只下发 `(vx, vy, wz)` | ⛔ **已被 0007 取代**（硬件前提消失） |
| [0002](0002-odometry-fusion.md) | `robot_localization` EKF 做单一 `odom→base_link`，分阶段融合四路里程计 | 已接受（架构；参数与源权重待实测标定） |
| [0003](0003-protocol-ownership.md) | 协议分层归属（CAN v3 归下位机 / UART 归上位机）与跨仓变更流程 | 已接受（8 项分歧待双方签字；CAN v3 层已失效） |
| [0004](0004-perception-compute-allocation.md) | 感知算力分配：DeepStream 9.1 可用 / Holoscan 4.6 不可用 / 重规划与 GUI 外移 | 已接受 |
| [0005](0005-grasping-stack.md) | 抓取栈分三阶段：几何优先 → GraspNet → VLM/AnyGrasp 远程 | 已接受（实现选型待阶段 1 实测） |
| [0006](0006-3d-obstacle-avoidance.md) | 3D 避障用 STVL；SLAM 保留 slam_toolbox（3D 来自深度相机） | 已接受（选型；参数与实测待闭合） |
| [0007](0007-unified-ros2-control.md) | 全机统一 `ros2_control`（`mecanum_drive_controller` + `joint_trajectory_controller`），麦轮 IK 移到上位机 | 已接受（**取代 0001**；「USB 是唯一链路」部分被 0012 取代） |
| [0008](0008-end-effector-and-perception.md) | 末端 = 标准 6 轴 + **舵机夹爪**（第 7 个 PWM 舵机）+ 末端相机；感知 = 深度相机 + 2D 雷达 + 语音 | 已接受（形态已定，硬件未成形） |
| [0009](0009-urdf-modeling-and-calibration.md) | 无 CAD 时用「参数化 + 图元近似 + chArUco 标定」建 URDF | 已接受 |
| [0010](0010-jev-harness-arbitration-and-expression.md) | Jev + Harness 任务仲裁：新包 `src/rak_harness` 作传递式仲裁缝；云端 LLM 走独立 `anthropic` 客户端；「呼吸感」分层预留接口 | 已接受（2026-10-07） |
| [0011](0011-wbc-hqp-motion-arbitration.md) | 全身运动仲裁层 WBC/HQP：独立节点 `rak_wbc`（ament_python）速度级 HQP（Pinocchio + OSQP），底盘与臂单一运动写权，Harness 传递式融合 | 已接受（2026-10-08；实现与硬件前置见待闭合项） |
| [0012](0012-transport-session-can-v4-primary.md) | 传输层 Session 抽象：CAN v4 唯一主链路、USB v2 冻结备用、`transport:=usb\|can` 工厂；**部分取代 0007 的"USB 唯一链路"** | 已接受（2026-10-09；USB 冻结备用部分被 0016 取代，协议内容部分被 0021 取代） |
| [0013](0013-workspace-subsystem-regroup.md) | 工作空间按子系统归堆 23→18 包：`rak_hardware`→`rak_chassis`、`rak_commissioning`/`rak_intent` 新建、bringup 三合一+navigation 并入、brain→`src/rak_agent`、CAN v3 栈入 `attic/`；"冻结消息契约"定为分层规则 | 已接受（2026-10-09） |
| [0014](0014-camera-contract.md) | RGB-D 相机契约：深度流（`/camera/depth/image_raw`+`camera_info`）必修、彩色可选；单一后端 `camera:=astra\|gemini335l` 参数切换；SDK 隔离；`eye/*` 立体相机不并入；外参不得臆造 | 已接受（2026-10-10） |
| [0015](0015-frame-tree.md) | 全机坐标系分层：REP-120 body 树（`base_footprint` 地面 / `base_link` 轮系中心）+ 无界 `world` 根 + 传感器/机械臂挂载帧；几何单一来源=`rak_chassis.urdf.xacro`，每帧唯一发布者 | 已接受（2026-10-11） |
| [0016](0016-retire-usb-transport.md) | 删除 USB v2 链路：CAN v4 唯一传输；工具只经 topic/action/service 与驱动节点对话，禁止直连硬件；`transport`/`device` 参数、`rak_base_driver`、`rak_serial_link`、直连 TUI、串口脚本与 `docs/protocol/` 全部删除 | 已接受（2026-10-14；协议内容部分被 0021 取代） |
| [0017](0017-read-feedback-vs-cmd-stale.md) | `read()` 反馈否决集剔除 `cmd_stale`(STATE.bit3)：反馈存活性与命令路径状态解耦，拆掉"ERROR→停用→置位→ERROR"自锁；`kStateFaultMask`/诊断不动 | 已接受（2026-10-14） |
| [0018](0018-tiered-realtime-voice-response.md) | 语音响应分层：即时反应层（预合成语气素材 <100ms）+ qwentts.cpp 分句流式（TTFA ≤1.5s）+ 显式响应预算；定稿 serena 音色 + instruct 情感 | 接受（2026-10-17 实施；实施机制与原方案有一处偏离，见文末「落地记录」） |
| [0019](0019-per-board-hardware-split.md) | 双下位机按板拆成两个 `<ros2_control>`（RakChassisHardware / RakArmHardware），共享一个 CanSession；故障域隔离，一块板掉线不再拖垮另一块的控制器 | 已接受（2026-10-29） |
| [0020](0020-managed-lifecycle-and-fault-domains.md) | 全栈受管生命周期：机制=标准 lifecycle_msgs、策略=`rak_lifecycle`+`system_manifest.yaml` 八故障域、观测=diagnostic_msgs；一个组件一个生命周期所有者；激活 ≠ 运动许可（`allow_motion` 仍是唯一许可） | 已接受（2026-10-29；修订 (a)：v5/CANopen 改变了 `link` 域；修订 (d)：全部自有节点转 managed） |
| [0021](0021-can-v5-canopen-on-mcus.md) | 下位机 CAN 换成 v5=CANopen（CiA 301+304，现成栈+EDS 单一来源+官方导出器）；本仓 v4 `CanSession` 对 R73/A14 板子失效，真机链路待 P3 换 lely-core 主站；禁止同总线混跑 v4/v5 | 已接受（2026-10-29；`rak` 侧实施未开始，真机链路当前不可用） |
| [0022](0022-jetson-canopen-contract-layer.md) | Jetson 侧 CANopen v5 架构：**契约层与协议栈分离**（生产主站=lely-core，本层只做 SSOT 派生与校验）；单位/因子单表带出处；"禁止 v4/v5 混跑"变成 COB-ID 交集断言（实测冲突 0x201/0x202）；NMT↔生命周期映射；Operational ≠ 运动许可 | 已接受（设计+契约层实现；生产主站实施待 P3） |
| [0023](0023-time-base-and-alignment.md) | 时间基准统一：采样时刻打戳、单一时钟域、协方差必须带出处 | 已接受（策略；Allan/直行回归数值待实测替换） |
| [0024](0024-canopen-master-on-jetson.md) | Jetson 侧 v5 主站走 ros2_canopen（Jazzy），lely-core 降为备选 | 已接受（**未编译、未实机验证**；修订 (a)：v4 整体下线） |
| [0025](0025-behavior-layer-bt-and-smacc2.md) | 行为层两级：BT.CPP 做技能、SMACC2 做任务；分阶段落地 | 已接受（**阶段 1 未实现**） |
| [0026](0026-assembly-layer-split-and-build-profiles.md) | 组装层拆成 `rak_orchestration`（契约，纯 x86 可建）+ `rak_bringup`（硬件全栈）；构建档位变成数据 | 已接受 |
| [0027](0027-4d-fusion-navigation.md) | 4D 融合视觉导航 v2.0：EKF 四路输入 + rtabmap 视觉里程计 + 时序体素代价层 + SmacPlannerLattice omni | 已接受（**软件门禁全绿，真机未验证**） |
| [0028](0028-mcp-tool-surface.md) | 跨进程 MCP 工具面：`src/rak_mcp` 只做提案缝的第三个客户端，工具表/校验/上限全部复用执行器；底盘单一写者变成代码级互锁；`source=mcp` | 已接受（**mock 链路已验，真机未验**） |

> **当前有效架构 = ADR-0002 + 0003(归属与流程部分) + 0004 + 0005 + 0006 + 0007(传输部分被 0012 取代) + 0008 + 0009 + 0010 + 0011 + 0012(备用链路部分被 0016 取代) + 0013 + 0014 + 0015 + 0016 + 0017 + 0018 + 0019 + 0020 + **0021（取代 0012/0016 的协议内容部分）** + 0022 + 0023 + 0024 + 0025 + 0026 + 0027 + **0028（修订 0010/0020 的"无外部控制面"边界）**。
>
> ⚠️ **ADR-0021/0024 之后，本仓 `rak_chassis` 的 v4 帧与 v5 板子不兼容**：真机联调走
> `transport:=canopen`（ros2_canopen 主站，ADR-0024）；台架临时主站是伞形仓的
> `tools/canopen_master.py`（更早的台架脚本 `tools/canopen_bringup.py` 见 ADR-0021），
> 二者都**不是**生产件。**v4 回滚分支已失效**（ADR-0024 修订 (a)，2026-10-30：R75/A16 已删掉
> v4 帧、`PROTOCOL.md` 与 `APP_USE_CANOPEN` 开关，`transport` 默认改 `canopen`），
> `rak_chassis` 的 `CanSession` 只剩软件夹具用途。
> **主站选型以 0024 为准**：0022 正文仍写 lely-core，同日 ADR-0024 已改为 ros2_canopen（Jazzy），
> lely-core 降为备选。
> 0001 仅作历史记录，不要按其实施。
> 0008 初稿曾误记末端为吸嘴，同日已更正为舵机夹爪（见该篇修订记录）。
> 0010 的 `src/rak_harness` / `harness_node` 已建（受管节点形态，见 ADR-0020 §5）；「呼吸感」硬件门全关。
> 0011 已接受（2026-10-08）；`src/rak_wbc` 骨架与桌面数值测试已建，但硬件接入的硬前置
> （`rak_hardware` 臂接口、臂质量与底盘几何实测、Jetson 侧 Python 依赖）未闭合前不上车。
> 任务与验收见 [`TODO.md`](../../TODO.md)。0004–0006 的结论来自 2026-10-05 的三路专项研究，
> **有时效性**，涉及版本矩阵/许可/内存的部分在实施前应复核（各篇"参考"节列了来源）。

## 勘误：旧路径与旧仓名（2026-11-14 登记，不改正文）

ADR 正文不可改，但仓结构与仓名变过，读到旧名字按本表换算：

| 正文里出现的旧名 | 现在的名字 |
|---|---|
| `docs/architecture.md` | `docs/explanation/architecture.md` |
| `docs/CONSTRUCTION.md` | `docs/tutorials/construction.md` |
| `docs/hardware_acceptance.md` | `docs/reference/hardware_acceptance.md` |
| `docs/arm_moveit_contract.md` | `docs/reference/arm_moveit_contract.md` |
| `docs/canopen-bitfields.md` | `docs/reference/canopen_bitfields.md` |
| `docs/migration.md` | `docs/explanation/migration_2026-10.md` |
| `docs/superpowers/specs/*` | `docs/design/*` |
| `docs/perf/*` | `docs/reference/perf/*` |
| `vehical_rak_chassis`（下位机底盘仓） | `vehical_rak_motion` |
| `vehical_rak_edge_ai`（下位机机械臂仓） | `vehical_rak_arm`（2026-11-14 改名） |
| `docs/CAN-BRINGUP-2026-10-30.md` | 从未存在过；对应内容见 `docs/status/next_2026-10-30.md` 与两块板仓的 `docs/CANOPEN-V5-DESIGN.md` |
| `docs/protocol/*`、`src/rak_base_driver`、`SerialSession` | ADR-0016 已删除，只在 git 历史里 |
