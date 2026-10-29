# rak_wbc — 全身运动仲裁（WBC/HQP）

> 决策依据 ADR-0011。本包把底盘与机械臂的**常规运动**收敛到**唯一写入者**：一个 100 Hz
> 速度级 HQP，输出底盘速度参考 + 臂关节位置流。安全性、可回归性和"谁在写电机"的边界
> 都优先于求解器的花哨程度。

## 1. 为什么存在

`mecanum_drive_controller` 与 JointTrajectoryController 各自都能直接驱动硬件；一旦同时
有 Nav2、MoveIt、遥操作、夹爪动作在写，就会互相打架。ADR-0011 把"常规运动"的仲裁权
交给本节点，其它写入者（MoveIt 轨迹、TUI 示教）只在**明确离权**时才动。

## 2. 关节命名与角度域（唯一口径，别再造第二套）

**2026-10-15/16 起全仓统一为"指令角域 + 硬件侧命名"**：运动学模型与硬件共用同一套
名字与同一套角度：

- 名字 = `j1_base … j6_wrist_roll`（= `<ros2_control>` / `joint_state_broadcaster` 的名字）。
- 角度 = **固件指令角**（0…270°；J2 硬件限 255°）。零位偏移（J2 +135°、J3 +180°）
  已**烘进 URDF 的 joint origin**（`rak_description/urdf/macros/arm_chain.xacro`），
  所以模型里的关节值本身就是指令角，**不再有 q_urdf ↔ 指令角 的换算层**。

因此本节点：`/joint_states` 直接按 `arm_joint_names` 取值即为 q_arm；下发的
`/wbc/arm_setpoints` 也是指令角，**同域直发**。⚠️ `arm_q_min/arm_q_max/home_q` 必须写
**指令角**值（旧 q_urdf 值会让臂停/走向错姿态：负角会被 Session「负值截 0」静默吃掉）。

## 3. 分层（数学与 I/O 分开，数学层可桌面回归）

| 文件 | 职责 | 依赖 |
| --- | --- | --- |
| `node.py` | ROS I/O + 状态机（idle/nav/servo/hold）、安全闸门、发布 | rclpy / numpy |
| `model_builder.py` | 组合 URDF（底盘 + 臂）→ Pinocchio 模型与运动学 | pinocchio |
| `hqp.py` | 速度级 HQP：把任务栈化成 QP，OSQP 求解 | osqp / numpy |
| `fake_arm_stream.py` | 软件臂夹具（仅测试/演示；不发真硬件） | rclpy |

## 4. 接口（与其它节点的契约）

- 订阅：`/cmd_vel`(Twist)、`/wbc/ee_target`(PoseStamped，带 frame)、`/wbc/mode`(String)、
  `/joint_states`(JointState)、TF(`map`→`base_link`)。
- 发布：底盘速度参考（`TwistStamped`，默认 `/mecanum_drive_controller/reference`）、
  臂位置流（`Float64MultiArray`，默认 `/wbc/arm_setpoints`，**指令角域** rad）、
  `/wbc/status`(String/JSON)。
- 服务：`/wbc/enable`(SetBool)。**注意 rclpy(Jazzy+) 的服务回调必须两参
  `(request, response)`**（单参签名会 TypeError 打死节点）。

接臂有两种写法（在 `rak_bringup` 里二选一，见 `chassis_bringup.launch.py` 的
`arm_motion_writer`）：`jtc` = MoveIt 的 `arm_controller`；`wbc` = 本节点 +
`arm_position_controller`（ForwardCommandController，`~/commands` 重映射到
`/wbc/arm_setpoints`）。**两者接口互斥，不能同时激活。**

## 5. 安全纪律（不可协商，见 `node.py` 头注）

- **默认 `enabled=false`**：不发任何运动命令；关闭时发一次零速后静默。
- 连续求解失败 ≥ `fail_limit` → `degraded`：停止发布，需显式重新 enable，**不自动恢复**。
- TF 丢失 → 发零速，不猜位姿。
- 不读 `/brain/decision`、不解析自然语言；目标必须带 frame。

## 6. 依赖与验证

- 运行期：`numpy` + `pinocchio`（PyPI `pin`）+ `osqp`。**rosdep 键未定**（见 package.xml
  注释与 ADR-0011 待闭合 #6）。
- 测试：`python3 -m pytest src/rak_wbc/test`（`conftest.py` 自带 sys.path，无需额外 PYTHONPATH）。
  **缺 pinocchio/osqp 的用例会自动 skip**（桌面无这两个库时只跑节点契约/命名类用例）。
  在 Jetson 上应全跑（`test_hqp_numerical.py` / `test_model.py` / `test_node_contract.py`）。
- 启动：`ros2 launch rak_wbc wbc.launch.py`（默认 `enabled=false`，另需 `publish_arm:=true`
  才发臂流；臂接口与实机几何仍是硬闸门）。

## 7. 仍未闭合（不要当已完成）

- `pinocchio`/`osqp` 在桌面缺席 → **HQP 数值层未在桌面跑过**，上 Jetson 必须先全跑测试。
- 臂的真实质量/几何/关节限位仍未全部实测回填（`wbc_params.yaml` 里标 `TODO(measured)` 的项）。
- `publish_arm` 默认关；臂位置流在实机（J1/J2 编码器电机 + J3..J6 舵机）上的闭环响应未验证。
