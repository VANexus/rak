# Rak 全身运动仲裁层（WBC/HQP）设计规格

> 状态：提案 · 最后更新：2026-10-08

- **日期：** 2026-10-08
- **作者：** yiyuhao
- **状态：** 已接受（ADR-0011）；骨架与桌面数值测试已建，硬件接入前置见 §10
- **范围：** `src/rak_wbc` 的数学模型、坐标系约定、ROS 契约与验收矩阵
- **前置文档：**
  - `docs/adr/0011-wbc-hqp-motion-arbitration.md`（决策与写权纪律）
  - `docs/adr/0007-unified-ros2-control.md`（执行层目标架构）
  - `docs/protocol/usb_serial_v2.md`（100 Hz 命令、lease、臂超时保持；**已按 ADR-0016 删除**，快照 tag `pre-usb-removal-20261013`）
  - `src/rak_description/urdf/properties_arm.xacro`（臂实测标定，单点定义）
  - `docs/design/2026-10-07-rak-arm-moveit2-design.md`（臂规划链，不重叠）

---

## 1. 概述与定位

`rak_wbc` 是底盘与臂之间的**速度级全身运动仲裁层**：每 100 Hz 解一个 9 自由度 QP，
输出（a）底盘机体速度 → `mecanum_drive_controller` reference，（b）臂关节位置流 →
位置控制器。它是常规运动的**唯一写入者**，但不是安全权威（ADR-0011 D5）：

```text
Nav2 /cmd_vel ─┐
/wbc/ee_target ─┼─▶ HQP(9 DOF, 100 Hz, OSQP) ─┬─▶ TwistStamped → mecanum reference
状态(TF+JS) ────┘                              └─▶ 臂位置流 → 位置控制器 → rak_hardware
```

数值层（`model_builder` + `hqp`）与 ROS 层（`node`）分离：前者不 import rclpy，桌面
pytest 可直接回归；后者只做 I/O 与状态机。

---

## 2. 事实分级

| 量 | 值 | 来源 |
|---|---|---|
| 臂限位/轴方向/零位/Home | 见 §5、§6 | **TUI 实测**（`properties_arm.xacro`，2026-10-08 第二轮） |
| 臂速度限幅 `[1.57, 0.5, 0.5, 1.57, 0.5, 1.57]` rad/s | J2/J3/J5 保守 0.5 | `properties_arm.xacro` 口径，**TODO(measured) 带载回填** |
| 臂质量/惯量 | 占位估重（各连杆 0.08–0.15 kg） | **占位值**，TODO(measured) |
| 底盘 `wheel_radius=0.0325 m`、`K=0.15 m` | 台架历史值 | `chassis_controllers.yaml` 自注占位，TODO(measured) |
| 底盘速度盒 `[0.25, 0.15, 0.5]` | 占位 | ADR-0006 记录的 Nav2 占位口径 |
| 支撑域半长/半宽 `0.15 m` | 占位 | 由 `K` 反推的保守值，TODO(measured) |
| 麦轮 `wheel_matrix` 符号 | 见 §6.3 | **TODO(verify)**：与 ros2_controllers 源码核对 |
| USB 100 Hz / lease 100 ms / 臂超时主动保持 | 协议规范（文档已按 ADR-0016 删除） | `docs/protocol/usb_serial_v2.md`（快照 tag `pre-usb-removal-20261013`） |
| 舵机 PWM 50 Hz | 固件/装配事实 | `app_servo.c` 口径 |
| Pinocchio 4.1.0 + OSQP 1.1.3 | 桌面 x86 实测可跑 | PyPI `pin` / `osqp` |

---

## 3. 模型与坐标系

### 3.1 组合模型（运行时合并，不复制几何源）

```text
wbc_world ──(wbc_x 平移)── wbc_x_link ──(wbc_y 平移)── wbc_y_link ──(wbc_yaw 旋转)── base_link
base_link ──(arm_mount_joint，fixed，占位安装位姿)── arm_base_link ──6×revolute── tool0
```

- `rak_wbc/urdf/wbc_base.urdf.xacro`：virtual 链 + `base_link`（占位质量）+ 安装关节。
- `model_builder` 用 xacro 展开 `wbc_base.urdf.xacro` 与 `rak_arm.urdf.xacro`，把后者的
  links/joints 并入前者，得到单一 URDF 字符串交给 Pinocchio。**臂几何只有一处定义。**

### 3.2 关节与变量顺序（按名字索引，不依赖 URDF 遍历序）

| 索引 | 变量 | 含义 |
|---|---|---|
| 0 | `wbc_x` | 世界系 x 平移速率 `ẋ_w` |
| 1 | `wbc_y` | 世界系 y 平移速率 `ẏ_w` |
| 2 | `wbc_yaw` | 世界系 yaw 角速率 `ψ̇` |
| 3–8 | `base_yaw` … `wrist_roll` | 臂关节速率 `q̇1…q̇6` |

**QP 决策变量 `v = [vx, vy, ωz, q̇1…q̇6]` 的底盘部分是机体坐标系**
（ROS REP-103：x 前、y 左、z 上；与 `/cmd_vel`、`mecanum_drive_controller` 语义一致）。
机体 ↔ 世界映射：

```text
[ẋ_w, ẏ_w, ψ̇]ᵀ = T(ψ) · [vx, vy, ωz]ᵀ,  T(ψ) = [ c −s 0; s c 0; 0 0 1 ]
J_ee_body(q) = J_ee_world(q) · blockdiag(T(ψ), I₆)        # 任务层统一在机体速度空间
```

### 3.3 末端与稳定相关量

- `tool0` 帧；`J_ee_world` 由 `pinocchio.computeFrameJacobian(..., LOCAL_WORLD_ALIGNED)`
  得到（6×9，世界对齐）。
- CoM：世界系 `c_w(q)`（Pinocchio）；**基座系** `c_b = R(ψ)ᵀ (c_w − p_base)`。
  其速率只由臂关节贡献（基座平面运动被精确抵消）：

```text
ċ_b = J_sup(q) · v ,  J_sup[:, 0:3] = 0
# 基座平移/偏航不改变基座系 CoM：平移是刚性跟随；偏航下 d(Rᵀ)/dψ·r 与
# Rᵀ·(dR/dψ)·c_b 两项精确抵消（有限差分测试断言）。只有臂构型会移动 CoM。
J_sup[:, 3:9] = R(ψ)ᵀ · J_com_world[0:2, 3:9]
```

链上 FK/Jacobian 均为微秒级；`J_sup` 的解析式与数值差分的等价性由桌面测试断言（§9）。

---

## 4. QP 总形式（v1：加权任务 + 硬约束）

```text
minimize   Σ_t w_t ‖A_t v − b_t‖² + λ‖v‖²
subject to  l ≤ C v ≤ u        （P0 硬约束，§6；结构恒定 42 行）
```

- `P = Σ w_t A_tᵀA_t + λI`（9×9，正定），`q_vec = −Σ w_t A_tᵀb_t`。
- 任务矩阵结构**恒定**：全部任务行常驻，靠权重调制启用/禁用（模式切换只改 `P/q`）。
- 求解器 OSQP，warm-start；失败/不可用路径见 §7。

---

## 5. 任务定义

| 优先级 | 任务 | `A_t` / `b_t` | 权重初值 |
|---|---|---|---|
| P1 | 末端跟踪 6D | `A = J_ee_body(q)`（6×9）；`b = [ẋ*, ω*] + [K_p(p*−p); K_R·log3(R_t R_curᵀ)]` | `w_ee_pos=100`、`w_ee_rot=20`；`K_p=2.0`、`K_R=1.0` |
| P2 | 底盘速度 | `A = [I₃ 0]`（3×9）；`b = v_cmd`（机体，来自 `/cmd_vel`） | `w_base_lin=10`、`w_base_yaw=5` |
| P3 | 姿态正则 | `A = [0 I₆]`（6×9）；`b = k_post(q_post − q_arm)`，`k_post=0.2` | `w_posture=0.2` |

**档位纪律与实测教训（2026-10-08 桌面闭环）**：P1:P2:P3 ≈ **100 : 10 : 0.2**，
数量级分离才是加权 HQP 的优先级表达。初版 `k_post=0.8 / w_posture=0.5` 时，姿态任务的
持续目标速度（≈0.6 rad/s）会把最优解推到冗余分支上与 P1 在零空间对抗——闭环误差停滞
甚至增长（实测收敛比：伸直构型 0.55、弯曲构型 1.05，即发散）；降到 `k_post=0.2 /
w_posture=0.2` 后为 0.09 / 0.17。**另注**：EE 跟踪的实际带宽被执行器限幅支配
（J2/J3 仅 0.5 rad/s，见 §10），桌面闭环在近伸直构型下即为此限幅掣肘。

- 新鲜度：`/cmd_vel` 超时 **0.3 s** → `b=0`（等效请求停车，仍走 P2）；`ee_target` 超时
  **0.5 s** → P1 权重置 0 并置状态位。
- 姿态目标默认 Home `q = [0, +90°, +90°, 0, 0, 0]`（ADR-0011 D4）。
- 方向误差用 `log3`（旋转向量，世界对齐）；奇异附近权重不归一（v1 不做 SVD 阻尼）。

---

## 6. 硬约束（P0，全部为线性不等式，`C` 共 42 行）

### 6.1 变量盒（18 行）

| 变量 | 界 |
|---|---|
| `[vx, vy, ωz]` | `±[0.25, 0.15, 0.5]`（占位，ADR-0006 口径；第一道线仍在 mecanum 控制器参数） |
| `q̇1…q̇6` | `±[1.57, 0.5, 0.5, 1.57, 0.5, 1.57]`（URDF `vel_fast/vel_slow`） |

### 6.2 关节限位速度屏障（12 行）

对每个臂关节 `i`：

```text
q̇ᵢ ≤  k_j · max(0, qᵢ_max − qᵢ)      q̇ᵢ ≥ −k_j · max(0, qᵢ − qᵢ_min)      k_j = 3.0 (1/s)
```

`max(0,·)` 使"已在限位内/外"时该侧上限为 0（只许离开限位方向），保证恒可行。
限位值（rad，URDF 口径）：
`[[0, 4.7124], [−2.3562, 2.0944], [−3.1416, 1.5708], [0, 4.7124], [0, 4.7124], [0, 4.7124]]`。

### 6.3 麦轮转速域（8 行）

```text
ω_wheel = W · [vx, vy, ωz]ᵀ ,  |ω_wheel,i| ≤ ω_max = 10.0 rad/s（URDF command clamp 口径）
W = (1/r) · [ 1 −1 −K ;  1  1  K ;  1  1 −K ;  1 −1  K ]      r = 0.0325, K = 0.15
```

**TODO(verify)**：`W` 符号与行序（FL/FR/RL/RR）以 ros2_controllers
`mecanum_drive_controller` 源码为准；`W` 只在配置中单点定义，核对后一处修正。
（语义说明：这是"求解器感知的轮域预限"，最终轮级等比饱和仍在 MCU，ADR-0007 D6。）

### 6.4 倾覆支撑域（4 行，CBF 形式）

支撑矩形 `|c_b,x| ≤ a − m`、`|c_b,y| ≤ b − m`（`a=b=0.15` 占位，`m=0.02`）。
对每个面：

```text
±J_sup[0]·v ≤ k_s · max(0, (a − m) ∓ c_b,x)      ±J_sup[1]·v ≤ k_s · max(0, (b − m) ∓ c_b,y)
k_s = 1.5 (1/s)
```

即"越接近边界，向外的允许速率越小；越界后禁止继续向外"，恒可行且不依赖里程计精度。

---

## 7. 求解与实时策略

| 项 | 设定 |
|---|---|
| 求解器 | OSQP（`eps_abs=eps_rel=1e-6`，`max_iter=4000`），CSC 稀疏，结构不变。**eps 收紧理由**：约束行量级 ≈30 rad/m（`1/r`），1e-4 设置会留下 ~4e-4 绝对残差（桌面实测），安全相关约束不接受该量级 |
| warm-start | **每周期 `setup()` + `warm_start(x, y)`**（缓存上一周期成功解的原/对偶变量）。不用 `osqp.update()`：osqp 1.1.3 上 `update(P=...)` 会静默失效（旧目标函数残留，桌面回归测试 `test_solver_matches_true_optimum_across_objective_change` 覆盖该历史缺陷），且 setup 剪除显式零使"模式恒等"无法保证。实测代价 0.34 ms 均值 / 0.49 ms 峰值（本机 x86；Jetson 复测入 §10） |
| 循环预算 | 目标 solve p50 < 1 ms、p99 < 3 ms（**桌面实测→Jetson 复测**，触发 C++ 迁移的判据） |
| 臂节拍 | 求解 100 Hz；臂位置流按 50 Hz 节拍发布（零阶保持对齐舵机 PWM） |
| 失败降级 | 求解失败 → `v=0`（停轮 + 臂保持）并计数；连续 3 次 → 停止发布（`reference_timeout` 0.5 s + 臂 lease 兜底）；恢复不自动 ARM |
| 非 RT 纪律 | 预分配数组、循环内不做非必要分配；逐周期 `setup` 是刻意取舍（见 warm-start 行，实测代价已量化），不假设 RT 内核 |

---

## 8. ROS 契约（`rak_wbc` 节点）

### 8.1 话题与服务

| 名称 | 方向 | 类型 | 语义 |
|---|---|---|---|
| `/cmd_vel` | 入 | `geometry_msgs/Twist` | P2 输入（Nav2 输出；机体系） |
| `/wbc/ee_target` | 入 | `geometry_msgs/PoseStamped` | P1 目标（**必须带 frame**，节点经 TF 变换到 `world_frame`；超时置零权重） |
| `/wbc/mode` | 入 | `std_msgs/String` | `idle \| nav \| servo \| hold`（§8.3） |
| `/wbc/enable` | 服务 | `std_srvs/SetBool` | 总开关；默认 **false**；关闭时发一次零速后静默 |
| `/mecanum_drive_controller/reference` | 出 | `geometry_msgs/TwistStamped` | 底盘速度（唯一常规写者；topic 可配） |
| `/wbc/arm_setpoints` | 出 | `std_msgs/Float64MultiArray` | 臂位置流（`publish_arm=false` 默认关闭；顺序 = `arm_joint_names`） |
| `/wbc/status` | 出 | `std_msgs/String` (JSON) | 10 Hz：`enabled/mode/solve_ms/status/fallbacks/support_margin/mode_bits` |
| `/wbc/posture_target` | 预留 | — | **当前不存在实现**（ADR-0011 D8.3 表达缝，门未开） |

### 8.2 关键参数（`config/wbc_params.yaml` 与 `node.py` 同源）

`enabled=false`、`mode=idle`、`rate_hz=100.0`、`world_frame=map`、`base_frame=base_link`、
`cmd_timeout=0.3`、`ee_timeout=0.5`、`publish_arm=false`、权重/增益/限位/几何见 §5–§6，
`fail_limit=3`，`arm_joint_names=[base_yaw_joint, shoulder_pitch_joint, elbow_pitch_joint,
forearm_roll_joint, wrist_pitch_joint, wrist_roll_joint]`。

### 8.3 节点状态机

```text
disabled(enabled=false) ──enable──▶ idle(静默) ──mode──▶ nav(P2+P3) / servo(P1+P2+P3) / hold(零速+保持)
   ▲                                                      │
   └──────────────── disable（发一次零速后静默） ◀────────┘
求解连续失败 ≥ fail_limit → degraded：停止发布 + status 提示；需显式重新 enable
```

### 8.4 Harness 融合（正方向，ADR-0011 D8 落地表）

| 来源 | 抵达方式 | HQP 可见形式 |
|---|---|---|
| `/harness/proposal` 的 `SELECT_WAYPOINT`/`REVISE_PLAN` | 执行器校验后提交 Nav2 goal | Nav2 输出 `/cmd_vel` → **P2** |
| 澄清/NO_OP | 无运动 | 不可见 |
| 未来操作类提案 | **须先版本化扩展 ADR-0010 契约**，再经新技能发 `/wbc/ee_target` | **P1** |
| 表达层（呼吸感） | 门控未开（ADR-0010 D4） | `/wbc/posture_target` 预留，仅 P3 |

**纪律**：Harness/云端永不发布 `/wbc/*`；HQP 不订阅 `/brain/decision`、不解析自然语言、
目标必须带 frame；`guard()`/E-stop/lease 均不在本层。

---

## 9. 验收矩阵

### 9.1 桌面（无 ROS；`src/rak_wbc/test`，venv：`pin` + `osqp`）

| 测试 | 断言 |
|---|---|
| 模型加载 | 组合 URDF 可被 Pinocchio 解析；9 关节按名索引；`tool0` 帧存在 |
| `J_ee` 数值 | 解析雅可比 vs 有限差分（位置/姿态），误差 < 1e-4 |
| `J_sup` 数值 | 同法验证，误差 < 1e-4 |
| 底盘跟踪 | nav 模式纯 `vx` 指令 → 解接近 `[vx,0,0,≈0…]` |
| 关节屏障 | 姿态目标越限时 `q̇ᵢ ≤ k_j·slack` 成立；限位外只许回程 |
| 轮域限幅 | 大横移指令下 `|W·v| ≤ ω_max` |
| 任务冲突 | `w_ee≫w_base` 时底盘偏离 `v_cmd`；反向权重时贴合 |
| 支撑域 | 支撑半长缩到 com 附近时，向外的 `vx` 被 CBF 压到 ≈0 |
| warm-start | 第二次解迭代数 ≤ 第一次且解一致（tol） |
| 降级 | 注入求解异常 → `v=0` + 状态位 + 计数 |

### 9.2 仿真（fake hardware / Gazebo）

脚本化 Nav2 速度 + 臂目标：末端跟踪误差、P1 借用 base 的可观测行为、倾覆约束触发降速。

### 9.3 台架（Arm Unlock Gates）

轮架空 + 臂 slow 单轴 → Home → 联合低速；软件暂停与物理急停分别验证；`mode=hold` 与
`enabled=false` 行为核对。

---

## 10. 待实测清单（占位 → 实测方法 → 影响）

| 占位值 | 实测方法 | 影响 |
|---|---|---|
| 臂质量/惯量 | 称重 + 质心悬线法 | 倾覆约束可信度（当前保守包络） |
| 底盘 `r`、`K`、支撑半长/半宽 | 台架滚轮标定 + 尺规 | 轮域限幅与支撑域 |
| `wheel_matrix` 符号 | 对照 ros2_controllers 源码 + 慢速单轴观察 | 轮域预限的正确性 |
| 底盘速度盒 | 实车安全测速 | P0 盒约束 |
| 求解 p50/p99（Jetson） | 节点内计时（status JSON） | C++ 迁移判据（ADR-0011 D1） |
| Jetson Python 依赖可得性 | `ros-jazzy-pinocchio` / `python3-osqp` / PyPI `pin` aarch64 | 部署路径（ADR-0011 待闭合 #6） |
| 50→100 Hz 零阶保持策略 | 台架阶跃响应对比 | 臂流平滑度 |