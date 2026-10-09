# Rak 真实六轴机械臂 MoveIt2 设计

- **日期：** 2026-10-07
- **作者：** yiyuhao
- **状态：** Pending review
- **范围：** `rak` 仓内 MoveIt2 对真实 6 轴机械臂的描述、规划与（仿真先行）执行链
- **前置文档：**
  - `docs/superpowers/specs/2026-10-03-rak-moveit2-integration-design.md`（5 轴旧版，保留作历史，被本文取代）
  - `docs/adr/0007-unified-ros2-control.md`（执行层目标架构）
  - `docs/adr/0008-end-effector-and-perception.md`（眼在手相机，本文不展开）
  - `docs/hardware_acceptance.md`（Arm Unlock Gates，真机执行的前置门禁）

---

## 1. 概述

为真实六轴机械臂建立 MoveIt2 规划栈。构型（用户 2026-10-07 确认）：

> 标准六轴：底座 360° 旋转 + 大臂上下 + 肘关节上下 + 小臂自旋 + 腕俯仰 + 腕圆周。

执行分两阶段：**阶段 A** 用 mock `ros2_control` 纯软件走通"规划 → FollowJointTrajectory → 仿真执行"全链；**阶段 B** 等 ADR-0007 的 `rak_hardware` SystemInterface 落地后，仅替换 controllers 配置即接真机（且必须先过 `docs/hardware_acceptance.md` 的 Arm Unlock Gates）。

夹爪（第 7 路 PWM 舵机开合）**不进 MoveIt 规划**，由独立的 gripper 接口控制，本文只保留 SRDF 占位组。

---

## 2. 硬件事实分级

按工作区规则区分"实测 / 转述 / 推断"：

### 2.1 实测/用户提供（TUI 探极限，2026-10-08 完成标定）

| 关节 | 舵机 | 类型 | 实测限位 | 零位（全直）指令 | 零位后限位（URDF） |
|---|---|---|---|---|---|
| J1 腰旋转 | MG995 | 位置型 | 0~270° | 0° | −90°~+180° |
| J2 大臂俯仰 | MG995 | 位置型 | **0~255°**（硬件约束） | 90° | −90°~+165° |
| J3 肘俯仰 | MG995 | 位置型 | 0~270° | 90° | −90°~+180° |
| J4 小臂自旋 | MG996R | 位置型 | 0~270° | 0° | −90°~+180° |
| J5 腕俯仰 | MG995 | 位置型 | 0~270° | 90° | −90°~+180° |
| J6 腕旋转 | MG996R | 位置型 | 0~270° | 0° | −90°~+180° |

标定过程与结论（2026-10-08）：
- **六轴全部为位置型舵机**。早前"J1/J4/J6 连续旋转"的判断已被实测推翻——R29/R40 阶段
  J1/J6"只动一点点"是位置舵机的脉宽死区（±100µs 偏置太小），不是速度模式；
  J4 当时确为连续旋转舵机，已更换为 996R 位置型。固件 R42 的 urad→脉宽映射对全部
  位置型统一成立，行为无需改；固件注释中"J1/J6 连续旋转"的说法待下次烧录时顺手修正。
- **方向关系（TUI 实测，用户确认）**：J2/J5 俯仰同向，**J3 反向**；全直姿态下
  三个自旋轴（J1 轴、J4 轴、J6 轴）共线，三个俯仰轴同向共面。
- **零位精度**：全直姿态为"肉眼直"（俯仰 ≈90°、自旋 ≈0°），非精标；URDF 的
  joint origin 与 SRDF home 待后续精标微调，不影响骨架正确性。
- 脉宽换算实测口径：500µs=0°、2500µs=270°，固件 R42 `car_arm_command_urad` 按此映射。
- 探极限时的指令超程（sent 范围越界部分）由固件 500~2500µs 钳位兜底，未损伤舵机。

**第二轮标定更新（2026-10-08，用户实测修订，已覆盖上表口径）：**
- 连杆长度修订：1-2:8cm、2-3:**10cm**、3-4:**11cm**、4-5:**6cm**、5-6:**9cm**。
- 零位指定（q_urdf=0）：自旋轴指令 0°；俯仰轴 **J2=135°、J3=180°、J5=0°**。
  由此 URDF 限位：J1/J4/J5/J6 = 0~+270°，J2 = −135°~+120°，J3 = −180°~+90°。
- 限速策略：大臂 J2、肘 J3 承重最大 → 0.5 rad/s（≈29°/s）保守起步；其余轴 1.57 rad/s。
  均为 TODO(measured)，带载实测后回填。
- 以上全部落入 `properties_arm.xacro`（单点定义）并由 `validate_rak_arm.py` 断言。
- **Home/初始/结束/安全位（用户 2026-10-08 指定）**：固件指令 J1=0°、J2=225°、
  J3=270°、J4=0°、J5=0°、J6=0° → SRDF `home` group_state = q(J2)=+90°、q(J3)=+90°、
  其余 0。TUI 的 Home 键与上电默认目标同为此姿态。

### 2.2 转述（待复核，进代码前须复核）

- MG996R 堵转力矩约 11 kg·cm @6V、MG995 约 9.4–10 kg·cm @6V（舵机公开规格书口径）。
  换算 ≈ **1.08 / 0.98 N·m**，URDF `effort` 用此值并标注来源。
- 角速度：舵机 0.17 s/60° 量级是空载数值；MoveIt 限速按保守 **90°/s（≈1.57 rad/s）** 占位，实测后替换。

### 2.3 推断（明标，不得当事实引用）

- **J1/J4/J6 的"360°"按连续旋转处理。** MG995/MG996R 标准版是 ±90° 行程；360° 意味着改装/连续旋转版或用户按总行程口径表述。URDF 先按 `continuous` 建，**首联真机前必须实测确认**——如果实际是普通舵机，`continuous` 会让规划器发出无限旋转指令。
- **3→4 段只有 5 cm、4→5 段 11 cm**：推断 3→4 是肘部支架偏置（轴向待定），4→5 是小臂主体（自旋轴沿其长度方向）。与常见的 6DOF 桌面机械臂（如 LeArm/XArm 类构型）一致，但**每段偏移沿哪个坐标轴、零位姿态是什么，全部未定**，靠 URDF 的 joint origin 逐项标注 TODO，零位标定时统一修正。
- J1 装于车顶：`base_link` 挂在整车 `base_footprint` 之上（z 偏移实测后填），MoveIt 侧先用 fixed virtual joint（底盘不动做规划）；随车移动规划是后续工作。

---

## 3. Goals & Non-Goals

### 3.1 Goals

1. `rak_description`：真实参数版 `urdf/rak_arm.urdf.xacro`（与 six_axis_sim fixture 并存、互不引用），全部几何/限位参数化，实测值入表、推断值标 TODO。
2. `rak_moveit_config`：`rak_arm.srdf` + `joint_limits.yaml` / `kinematics.yaml` / `ompl_planning.yaml` / `mock_controllers.yaml`（阶段 A）+ `controllers.yaml`（阶段 B 模板，先不启用）。
3. `six_axis_arm.launch.py`：阶段 A 全链启动（robot_state_publisher + move_group + mock hardware + RViz）。
4. 验证脚本 `test/validate_rak_arm.py`（xacro 解析 + SRDF 结构 + joint 名一致性），并入 `colcon test`。
5. 里程碑 M1–M4（同旧 spec 的四段验证链）全部在阶段 A 走通。

### 3.2 Non-Goals

- 不接真机（`rak_hardware` 不存在；Arm Unlock Gates 未过）。
- 不做夹爪规划（独立 PWM 舵机，简单开合接口即可）。
- 不改 six_axis_sim fixture 与 `rak_arm_simulation`（它们是测试夹具，保持软件仿真定位）。
- 不动底盘/Nav2。机械臂装在车顶，但本轮 virtual joint 用 fixed，不做随车规划。
- 不引入 IKFast/解析 IK；KDL 数值解起步。

---

## 4. 构型映射与命名

沿用 `rak_moveit_config` 六轴 fixture 已验证的关节命名（与 `rak_arm_simulation`、ADR-0007 的 7 舵机口径对齐）：

| 轴 | 关节名 | URDF 类型 | 限位（rad，零位=全直） | 来源 |
|---|---|---|---|---|
| J1 | `base_yaw_joint` | revolute | −π/2 ~ +π | 0~270°（TUI 实测） |
| J2 | `shoulder_pitch_joint` | revolute | −π/2 ~ +2.8798（+165°） | 0~255°（TUI 实测，硬件约束） |
| J3 | `elbow_pitch_joint` | revolute（轴 −Y） | −π/2 ~ +π | 0~270°；方向与 J2/J5 反向（实测） |
| J4 | `forearm_roll_joint` | revolute | −π/2 ~ +π | 0~270°（换 996R 后实测） |
| J5 | `wrist_pitch_joint` | revolute（轴 +Y） | −π/2 ~ +π | 0~270°（TUI 实测） |
| J6 | `wrist_roll_joint` | revolute | −π/2 ~ +π | 0~270°（TUI 实测） |

> 2026-10-08 更新：早期"J1/J4/J6 为连续旋转舵机（360°）"的口径已被 TUI 探极限实测推翻，
> 六轴均为位置型，详见 §2.1。命名 J4 沿用 `forearm_roll_joint`（小臂自旋）。

> 命名差异说明：旧 fixture 里 J4 叫 `wrist_yaw_joint`；真实构型中它是**小臂自旋**，改名 `forearm_roll_joint` 更准确。`rak_arm_simulation` 的夹具名不动（隔离），新 URDF 用新名，映射表写进 SRDF 注释。

连杆链（车顶安装，暂定零位"大臂竖直"为占位姿态，**待零位标定修正**）：

```
base_footprint (整车)
  └─ arm_base_link        车顶安装面 → J1 (z +0.08 TODO 实测)
      └─ shoulder_link    J1 底座旋转 (Z 轴)
          └─ upper_arm_link   J2 大臂俯仰 (Y 轴) — 大臂 0.11 m
              └─ forearm_link    J3 肘俯仰 (Y 轴) — 肘偏置 0.05 m
                  └─ forearm_roll_link  J4 小臂自旋 (沿小臂轴) — 小臂 0.11 m
                      └─ wrist_pitch_link  J5 腕俯仰 (Y 轴)
                          └─ wrist_roll_link  J6 腕旋转 (沿腕轴) — 0.07 m
                              └─ tool0 (法兰，约定 tip)
```

每段偏移沿哪个轴（X/Y/Z）依赖零位姿态定义，属 §2.3 推断项；`properties.xacro` 集中放全部数值，逐项 `<!-- TODO(measured): -->`。

---

## 5. 包与文件布局

```
src/rak_description/
  urdf/
    rak_arm.urdf.xacro          # 真实参数版（新增；不引用 six_axis_sim）
    properties_arm.xacro        # 本臂专属参数：长度/限位/质量，全部带来源注释
    （复用现有 macros/inertial_macros.xacro、materials.xacro）
src/rak_moveit_config/
  config/
    rak_arm.srdf
    rak_arm_joint_limits.yaml
    rak_arm_kinematics.yaml     # KDL：position_only_ik=false，timeout 0.05s
    rak_arm_ompl_planning.yaml  # RRTConnect 默认
    rak_arm_mock_controllers.yaml   # 阶段 A：mock ros2_control 的 JTC
    rak_arm_controllers.yaml        # 阶段 B 模板（rak_hardware + JTC + gripper），注释禁用
  launch/
    six_axis_arm.launch.py      # 阶段 A 全链
  test/
    validate_rak_arm.py         # xacro 解析 + 关节名/限位一致性断言
```

与 six_axis_sim 的隔离是硬要求（CLAUDE.md）：fixture 的 `validate_six_axis_sim.py`、`rak_arm_simulation` 均不改；新文件不 import、不 include fixture 的任何文件。

---

## 6. 关键设计决定

| # | 决定 | 理由 |
|---|---|---|
| D1 | J1/J4/J6 建 `continuous` | 用户口径 360°；但首联真机前必须实测确认舵机是否真连续旋转（§2.3），若是普通舵机改 `revolute ±π` 并加软件限位 |
| D2 | 270° 关节限位取 ±135° | 行程对称化占位；机械零位未标定，真实不对称限位待零位测试后修正 |
| D3 | effort 用 0.98/1.08 N·m、velocity 用 1.57 rad/s | 舵机规格转述值 + 保守降额；标注 TODO(measured) |
| D4 | 夹爪不进规划组 | PWM 舵机开合无位置反馈价值，走独立接口（ADR-0008）；SRDF 只留 `gripper` 占位组 |
| D5 | virtual joint = fixed | 车顶安装但本轮不做随车规划；底盘运动时机械臂应停持（安全策略另定） |
| D6 | KDL 起步 | 本构型非球形腕（roll-pitch-pitch-roll-pitch-roll），KDL 数值解 6DOF 足够；不可达姿态记录后评估 IKFast |
| D7 | 阶段 A 用 mock `ros2_control`（`mock_components/GenericSystem`）而非旧 fake_joint_driver | 与 ADR-0007 目标架构同构：阶段 B 只换 hardware plugin 与 controllers.yaml，launch/SRDF/限位全复用 |
| D8 | 零位未定：URDF 暂定"各关节 0 = 用户后续标定基准" | 所有关节 origin/轴向集中注释，零位标定是阶段 B 的前置项之一 |

---

## 7. 阶段 A：验证链（本轮交付）

沿用四里程碑：

| 里程碑 | 内容 | 验证命令（要点） |
|---|---|---|
| M1 | xacro 解析通过 | `ros2 run xacro xacro .../rak_arm.urdf.xacro` + `check_urdf` |
| M2 | TF 树完整（9 link） | `view_frames.py` 生成树图，链路无断点 |
| M3 | move_group 可规划 | RViz 拖动 InteractiveMarker；CLI `ros2 action send_goal /plan_kinematic_path` |
| M4 | 轨迹执行闭环 | mock hardware 执行 `/arm_controller/follow_joint_trajectory`，`/joint_states` 随之变化 |

测试集成：`colcon test --packages-select rak_moveit_config rak_description` 跑 `validate_rak_arm.py`；`validate_six_axis_sim.py` 原样保留并继续通过。

## 8. 阶段 B（前置条件，不在本轮）

1. `rak_hardware` SystemInterface 落地（ADR-0007），7 路位置命令（6 关节 + 1 夹爪）走 USB。
2. **零位标定完成**：每个关节的机械零位 ↔ 指令角度映射，回填 URDF origin 与 SRDF `group_state`。
3. **J1/J4/J6 连续性实测**（D1 风险闭合）。
4. 关节速度/加速度限位实测回填 `rak_arm_joint_limits.yaml`。
5. Arm Unlock Gates（`docs/hardware_acceptance.md`）通过。
6. 切换动作 = 只改 launch 参数：hardware plugin `mock_components/GenericSystem` → `rak_hardware/RakSystem`，controllers 换 `rak_arm_controllers.yaml`。

---

## 9. 风险

| 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|
| MG995/996R 实为 ±90° 普通舵机，360° 口径是总行程误读 | 中 | 高（continuous 会规划出无限旋转） | D1 实测前 mock 阶段无害；真机切换前强制实测（§8.3） |
| 5 cm 段轴向判断错误 → IK 不可达/奇异 | 中 | 中 | 偏移轴向全部参数化，零位标定时一次修正；M3 记录不可达姿态 |
| MG996R 带载力矩不足（大臂 11 cm + 载荷） | 中 | 高（抖动/掉步） | 属硬件问题：限速保守 + 阶段 B 前带载实测；MoveIt 侧加速度限位再降 |
| KDL 对该构型部分姿态无解 | 中 | 中 | 记录失败位姿；必要时 position_only_ik 或换解析 IK |
| Orin 内存压力（move_group + Laya 并存） | 低 | 中 | 不并行启动；`rak-mode.sh` 已有内存档位管理 |

---

## 10. 未决问题（不阻塞本轮）

1. J2/J3/J5 的 ±135° 是否真实对称（舵机中位与机械中位的关系，等零位标定）。
2. `tool0` 之后到夹爪根部的距离（夹爪设计未定）。
3. 机械臂在车顶的具体安装位姿（影响未来随车规划与 `arm_base_link` 的 z 偏移）。
4. 眼在手相机（ADR-0008）的安装 link，待相机支架确定后加入 URDF。
