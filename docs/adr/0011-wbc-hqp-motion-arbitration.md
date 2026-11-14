# ADR-0011: 全身运动仲裁层 WBC/HQP —— 独立节点、速度级、底盘与臂的单一运动写权

- **状态**: 已接受（2026-10-08 用户确认"底盘 + 臂一次到位"，并要求融合 Harness 架构）
- **日期**: 2026-10-08
- **相关**: `src/rak_wbc`（本 ADR 创建）· `src/rak_hardware` ·
  `src/rak_chassis_bringup/config/chassis_controllers.yaml` ·
  `src/rak_description/urdf/rak_arm.urdf.xacro` · `src/rak_description/urdf/properties_arm.xacro` ·
  `docs/protocol/usb_serial_v2.md` · `docs/design/2026-10-08-rak-wbc-hqp-design.md` ·
  ADR-0002 · ADR-0004 · ADR-0006 · ADR-0007 · ADR-0008 · ADR-0010

---

## 背景

### 已核实事实（2026-10-08 直接查代码/提交）

- **底盘链路已落地**：`src/rak_hardware`（`hardware_interface::SystemInterface`，当前只导出
  4 轮接口）+ `src/rak_chassis_bringup`（100 Hz `controller_manager` +
  `mecanum_drive_controller` + `joint_state_broadcaster`）已通过 F407 实驱烟测（提交
  `f5ad2c2`，固件 R37）。`allow_motion` 默认 false；`reference_timeout: 0.5` 是"HQP 停发后
  自动停轮"的安全网。
- **真实的六轴臂已实测标定**（提交 `64dfa52`/`80ec83f`）：`rak_arm.urdf.xacro` +
  `properties_arm.xacro`（连杆长度、J1~J6 限位、轴方向、零位映射、Home 安全位
  `q = [0, +90°, +90°, 0, 0, 0]`），`validate_rak_arm.py` 断言以上全部。
- **臂的已知限制**：位置舵机、**无关节位置反馈**（`ARM_STATE` measured 不可用，TUI 只读
  mode/fault/echo）；速度限幅保守（J2/J3/J5 = 0.5 rad/s，其余 1.57 rad/s，TODO(measured)）；
  **质量/惯量是占位估重**；夹爪（第 7 路 PWM）未挂 `tool0`；F407 舵机 PWM 为 50 Hz。
- **执行器语义**：轮 = 速度接口、臂 = 位置接口，**没有任何力矩接口**；`rak_hardware` 的臂
  位置接口尚未导出；USB v2 命令速率 100 Hz，lease 10–200 ms，臂 lease 超时"主动保持"。
- **桌面依赖实测**：PyPI `pin`（Pinocchio 4.1.0）+ `osqp`（1.1.3）在 x86 + Python 3.14 的
  独立 venv 中安装并跑通 —— 数值原型不需要 Jetson 即可验证。
- **既有权威划分**：`mecanum_drive_controller` 只做机体轴限幅（轮级等比饱和在 MCU，
  ADR-0007 D6）；EKF 是唯一 `odom→base_link` 发布者（ADR-0002）；`guard()` 是唯一安全权威、
  Harness 只出提案（ADR-0010）；`/cmd_vel` 的最终写者尚未接线。

### 问题

机体只有一个，运动写者有两个：底盘控制器（未来由 Nav2 填充）与臂的控制器。没有任何组件
同时看到两边——臂伸出时底盘照旧全速（**倾覆是真实物理风险**）；底盘运动时末端位姿无人保持；
导航与操作目标冲突时无人仲裁。这是移动操作平台的标准缺口，也是"全身控制（WBC）"在本平台
要兑现的东西。

**本平台"全身控制"只能是速度级运动学**：没有力矩接口，100 Hz 链路也没有动力学回路预算；
加速度/力矩级 WBC 只可作为仿真研究基线，不能承诺上车。

---

## 决策

### D1. 形态：新包 `src/rak_wbc`（ament_python），节点 `wbc_node`，100 Hz 速度级 HQP

- **控制层级**：速度级运动学；决策变量 `v = [vx, vy, ωz, q̇1…q̇6] ∈ R⁹`
  （**机体坐标系**：麦轮底盘平面全向 3 DOF，无奇异；与 `/cmd_vel` 语义一致）。
- **v1 语言选 Python（ament_python）**，运行在 ROS 系统 Python，不建 venv：
  1. 仓库已有 100 Hz Python 控制环先例（`rak_base_driver` 命令 tick、`chassis_key_test`）；
  2. Pinocchio / OSQP 的 Python 绑定是一等公民，且桌面数值原型已实测可跑（见背景）；
  3. 100 Hz = 10 ms 预算，9 变量 QP 求解加 FK 在 Python 上有数量级余量。
  **迁移判据（重评 C++ 的唯一条件）**：实测 solve + 循环 p99 超过 3 ms，或需要进程内
  挂载为 `ros2_control` 控制器（控制器链式化）时，重开 ADR 评估 C++ 移植；当前不预支。
- **频率**：100 Hz 固定步长，与 USB v2 命令节奏对齐；臂位置流按 50 Hz 节拍对齐舵机 PWM
  帧率（零阶保持，spec §7）。
- **输出**：
  1. `geometry_msgs/TwistStamped` → `mecanum_drive_controller` 的 `~/reference`
     （**HQP 是该 reference 的唯一常规写者**；topic 名/类型以 Jazzy 控制器参数为准）；
  2. 臂位置流 → ros2_control 位置控制器 → `rak_hardware` 臂接口（前置条件见 D7）。

### D2. 任务栈：v1 全量覆盖底盘 + 臂（用户 2026-10-08 裁定"一次到位"）

| 优先级 | 任务 / 约束 | 数学形式 |
|---|---|---|
| **P0 硬约束** | 关节限位速度屏障 · 速度盒 · 麦轮转速域 · 倾覆支撑域 | 线性不等式（盒 + CBF 式） |
| P1 | 末端位姿跟踪（`tool0`） | `J_ee(q)·v = ẋ* + K·e` |
| P2 | 底盘速度跟踪（Nav2 `/cmd_vel` 为输入；P1 缺自由度时可借用） | `v_base ≈ v_cmd` |
| P3 | 姿态正则（回 Home、远离关节限位） | `q̇ → k(q_home − q)` |

- v1 用**加权 QP**（权重显式配置在 `config/wbc_params.yaml`）；**级联（严格字典序）
  接口预留**，任务复杂度增长后再切换。逐条公式见 spec §4–§6。
- P1 是速度级 resolved-rate 控制，**不做 IK**：全局/避碰规划仍归 MoveIt（远程，ADR-0004），
  本层只做伺服级跟踪。

### D3. 数学与工具

- **组合模型**：运行时由 `rak_wbc/urdf/wbc_base.urdf.xacro`（`wbc_world` + 3 平面虚拟关节
  x/y/yaw + `base_link` + 臂安装关节）与 xacro 展开后的 `rak_arm.urdf.xacro`
  **合并为单一 URDF 字符串**交给 Pinocchio —— 不复制任何几何源，臂只有一处定义。
- 基座变量到世界系（Pinocchio 平面关节）的映射在模型层完成，见 spec §3。
- 循环纪律：**零动态分配 + OSQP warm-start + 超时降级**；不假设 RT 内核。
- 本层内存增量数十 MB 量级，不与 ADR-0004 的 DeepStream 预算冲突。

### D4. 臂的接入与模式仲裁

- 臂有两种**互斥**写入模式，由上层技能层仲裁：**轨迹模式**（MoveIt 远程规划 → JTC，
  ADR-0007 现状）与**伺服模式**（HQP 位置流，本 ADR 新增）。
- v1 落地伺服模式 + Home 安全位；`publish_arm` 默认 false，直到臂接口/控制器就绪。
- 臂位置流按 URDF 限值子集限幅；依赖固件 100 ms lease 的"超时主动保持"语义。

### D5. 写权与安全纪律（不可协商）

1. **HQP 是常规运动的唯一写入者**：Nav2 不直连 `mecanum_drive_controller` reference；
   `/cmd_vel` 成为 HQP 的 P2 输入；臂位置流的常规写者同样是 HQP。
2. HQP **不订阅** `/brain/decision`、不接受自然语言目标；只接受带坐标系的结构化目标。
3. **不绕过任何既有权威**：`allow_motion`/DISARMED/E-stop/lease/`guard()` 全部保持；
   机体轴限幅仍是第一道线。
4. **失败故事闭合**：HQP 求解失败 → 输出零速（停轮 + 臂保持）并计数；连续失败 →
   停止发布（`reference_timeout` 0.5 s 兜底 + 臂 lease 保持）；恢复后**不自动重新 ARM**。

### D6. 验收（三级，对齐 `docs/reference/hardware_acceptance.md` 的门槛风格）

1. **桌面（无 ROS、纯数值）**：约束激活、任务冲突仲裁、warm-start、降级路径、模型
   FK/J_ee 数值健全性 —— `src/rak_wbc/test`（本 ADR 已附带可运行版本）。
2. **仿真（fake hardware / Gazebo）**：脚本化 Nav2 速度 + 臂目标；末端跟踪误差、P1 借用
   base 的可观测行为、倾覆约束触发降速。
3. **台架（依 Arm Unlock Gates）**：轮架空 + 臂 slow 单轴 → Home → 联合低速；
   软件暂停与物理急停分别验证。

### D7. 实现顺序 ≠ 范围收缩

v1 范围"底盘 + 臂一次到位"，施工有技术前置：
① `rak_hardware` 导出臂位置接口 + 位置控制器（含 Home）→ ② HQP 臂流上车；
底盘侧（P0/P2/P3）无前置，可先行上车。
v1 **不做**（列 v2/v3）：自碰撞约束、FOV 保持任务、动力学项、移动抓取技能（TMC 编排）。

### D8. Harness 融合：定义 HQP 的"正方向"契约（2026-10-08 用户要求）

Harness（ADR-0010）与 HQP 之间**没有直连**，只有传递式正方向链路；本节把这条链路和
落点话题一次性定义清楚，防止将来临时发明：

```text
/harness/proposal ──▶ rak_task_executor（唯一生命周期所有者，校验 task_id/revision/expires_at/白名单）
                         │  技能层（导航 / 未来的操作技能）
                         ▼
                     HQP 目标话题（本 ADR 定义）
    /cmd_vel(FROM Nav2, P2) · /wbc/ee_target(P1) · /wbc/mode · /wbc/enable
                         ▼
                      rak_wbc ──▶ mecanum reference（TwistStamped）/ 臂位置流
```

1. **提案 → HQP 的映射（只读表）**：
   | Harness 提案 | 抵达 HQP 的方式 |
   |---|---|
   | `KIND_SELECT_WAYPOINT` / `KIND_REVISE_PLAN` | 执行器提交 Nav2 goal → Nav2 输出 `/cmd_vel` → HQP **P2**。Harness 永不直发速度 |
   | `KIND_REQUEST_CLARIFICATION` / `KIND_NO_OP` | 无运动。HQP 不可见 |
   | （未来）操作类提案（如"抓取"） | **不在本 ADR**：须先版本化扩展 ADR-0010 契约 → 新技能 → `/wbc/ee_target`。未扩契约前不存在该路径 |
2. **HQP 侧接口契约**（spec §8 全表）：`/cmd_vel`(Twist, P2 输入) · `/wbc/ee_target`
   (PoseStamped, P1 输入, 必须带 frame) · `/wbc/mode`(`idle|nav|servo|hold`) ·
   `/wbc/enable`(SetBool) · `/wbc/status`(String JSON) · `/wbc/arm_setpoints`(Float64MultiArray)。
   **Harness 永远不发布上述任何话题**；发布者只有 Nav2、技能层、操作者工具。
3. **「呼吸感」表达层预留缝（不实现）**：`/wbc/posture_target` 为**预留**话题（当前不存在
   实现）。未来若 ADR-0010 的 `ExpressionGate` 放开，`IDLE_POSTURE / ORIENT_ATTENTION` 只能
   映射为 **P3 姿态正则的目标**，且必须满足：无活动控制流占用资源、底盘未 ARM 或人工接管
   优先、不叠加到活动导航/抓取（ADR-0010 红线逐字保留）。WBC 不得在无 ADR 的情况下成为
   表达执行器。
4. **纪律重申**：HQP 不读 `/brain/decision`、不读 ASR/提案原文；它只吃结构化目标 +
   状态 + 配置。Harness 的"智能"对 HQP 不可见，正如 `guard()` 的权威对 Harness 不可见。

---

## 理由

1. **独立节点改动面最小**：保留 ADR-0007 全部既有语义（标准里程计/协方差、
   `reference_timeout` 保底、机体轴限幅），HQP 可单独扩容/回滚。
2. **Python 起步是"先验证数学"的最短路径**：数值层与 ROS 层可分离，桌面即可回归；
   ROS 系统 Python 已有大量先例；性能判据（D1）明确，不赌。
3. **速度级是执行器唯一可兑现的层级**（无 τ 接口）。
4. **单一写权消灭双写者**，并把倾覆从工程直觉变成数学约束（P0）。
5. **D8 让 Harness 有正方向落点**：提案 → 执行器 → 技能 → 目标话题的传递链，既保住
   "提案不赋权"，又让 Harness 的意图最终能影响物理运动。

---

## 后果

### 正面

- 底盘与臂的冲突有了数学仲裁点；倾覆风险进入 P0 约束。
- 数值层桌面可回归（本 ADR 附带测试），迭代快、回归早。
- 失败故事闭合：HQP 死 → 停轮 + 臂保持，不产生新的失控路径。
- Harness 的意图有了明确、可审计的物理落点（经执行器/技能，非直连）。

### 负面（代价，必须承认）

- **Python 抖动风险**：非 RT + GC；缓解是零分配/预分配 + warm-start，判据见 D1。
  若判据触发，迁移 C++ 是真实成本。
- **依赖可用性**：PyPI `pin` 的 aarch64 轮子/`ros-jazzy-pinocchio` 的 Python 绑定、
  `python3-osqp` 在 Jetson 上的可得性**待核实**（待闭合 #6）；不可得则需源码构建或策略调整。
- **臂无反馈**：HQP 的臂状态是命令回读（乐观），末端误差与倾覆约束在负载下不可信；
  闭环靠视觉（eye-in-hand），这是设计约束不是待修 bug。
- **质量/底盘几何未实测前**，倾覆约束按保守包络运行（牺牲可用速度）。
- 舵机 50 Hz PWM + 机械带宽：P1 只做慢速协同。
- 模式仲裁（JTC/伺服）与权重矩阵都是新调参面。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 首版就用 C++ 节点 | 构建面（colcon+pinocchio/osqp C++ 依赖）压过收益；桌面上无法跑数值回归；100 Hz 预算富余，性能不是当前瓶颈。**迁移判据已写入 D1**，不是永久否决 |
| HQP 作为 `ros2_control` 控制器进程内运行 | 需接管轮速接口并自行等效里程计，改动面大；非 RT 系统进程内收益有限。留作后续演进 |
| HQP 直写 4 轮速度接口（绕过 `mecanum_drive_controller`） | 丢标准里程计/协方差、`reference_timeout` 保底与机体轴限幅 |
| 动力学 / 力矩级 WBC 上车 | 无 τ 接口，硬件上无执行路径；仅可作为仿真研究基线 |
| HQP 永久取代 JTC | 丢失轨迹语义（时间参数化、与 MoveIt 对接）；双模式更符合生态 |
| Harness（或任何云端输出）直发 `/wbc/*` 或速度/关节 | 违反"提案不赋权"与单一写权；D8 已定义唯一传递链 |
| HQP 订阅感知/决策话题直接决策 | 违反 `guard()` 唯一权威纪律；感知进 100 Hz 环也没有带宽/预算（ADR-0004/0006） |
| 底盘保持 Nav2 直连、HQP 只管臂 | 保留双写者与倾覆风险，正是本层要解决的问题 |
| 复制一份臂几何进 WBC 专用 URDF | 双源漂移；改为运行时合并（D3） |

---

## 待闭合项

1. **`rak_hardware` 臂位置接口未导出**（硬前置）；臂位置控制器具体类型需核实。
2. **臂质量/惯量实测**（当前占位估重）→ 决定倾覆约束可信度。
3. **底盘真实几何**（wheelbase/track/wheel radius 现为占位）→ 麦轮转速域与支撑域。
4. **臂无关节反馈**：命令回读语义标注；视觉闭环路径单列（eye-in-hand）。
5. **夹爪（第 7 轴）**与 `tool0` 挂载、角度↔开度映射（ADR-0008）——v1 不含夹爪动作。
6. ✅ **Jetson 侧 Python 依赖可得性（2026-10-08 闭合）**：`ros-jazzy-pinocchio` 4.1.0
   （apt，含 python3 绑定）+ `osqp` 1.1.3（`pip --user --break-system-packages`，
   仅新增 cloudpickle/joblib/osqp 三个小包）在 Orin Nano (aarch64, Jazzy) 上实测可用。
7. **麦轮映射系数符号**：`wheel_matrix` 与 ros2_controllers 源码核对（配置单点定义，便于修正）。
8. ✅ **Python 循环 p50/p99 实测（2026-10-08 闭合，判据未触发）**：Jetson 真机 nav 模式
   25 s 采样 657 次：**p50=2.41 ms / p99=2.85 ms / max=4.90 ms**（单次 QP solve）——
   p99 < 3 ms，D1 的 C++ 迁移判据**未触发**，Python 方案在设备上成立；
   100 Hz 预算（10 ms）占用 <50%。（x86 对照 0.34 ms；servo 模式与全环时序待上车复测。）
9. **50 Hz PWM 与 100 Hz 求解的零阶保持策略**实测。
10. **模式仲裁状态机**（JTC/伺服）细节，待真臂 MoveIt 配置就绪。
11. **文档基线同步**：`TODO.md` P0-1/`docs/status/project_status.md`/`docs/explanation/architecture.md` 包表
    的陈旧条目（本次已同步 `AGENTS.md`/`CLAUDE.md` 关键事实与 `architecture.md` L2 描述）。
12. **验收脚本**（桌面/仿真/台架）与 `docs/reference/hardware_acceptance.md` 的对接条目。

---

## 参考

- 设计规格（逐条公式）：`docs/design/2026-10-08-rak-wbc-hqp-design.md`
- `src/rak_hardware/include/rak_hardware/rak_system.hpp`、`src/rak_chassis_bringup/config/chassis_controllers.yaml`
- `docs/protocol/usb_serial_v2.md`（100 Hz 命令、lease、臂超时保持）
- `src/rak_description/urdf/rak_arm.urdf.xacro`、`properties_arm.xacro`、
  `src/rak_moveit_config/config/rak_arm.srdf`、`test/validate_rak_arm.py`（实测标定）
- ADR-0002/0004/0006/0007/0008/0010
- Pinocchio（PyPI `pin`，桌面实测 4.1.0）：<https://github.com/stack-of-tasks/pinocchio>
- OSQP（Python API，桌面实测 1.1.3）：<https://osqp.org>
- 层级 QP 任务优先级：Kanoun et al., *Kinematic Control of Redundant Manipulators:
  Generalizing the Task-Priority Framework to Inequality Tasks*, IEEE T-RO 2011