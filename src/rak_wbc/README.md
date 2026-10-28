# rak_wbc — 全身运动仲裁（WBC/HQP）

> 决策依据 ADR-0011。本包把底盘与机械臂的**常规运动**收敛到**唯一写入者**：一个 100 Hz
> 速度级 HQP，输出底盘速度参考 + 臂关节位置流。安全性、可回归性和"谁在写电机"的边界
> 都优先于求解器的花哨程度。

## 1. 为什么存在

`mecanum_drive_controller` 与 JointTrajectoryController 各自都能直接驱动硬件；一旦同时
有 Nav2、MoveIt、遥操作、夹爪动作在写，就会互相打架。ADR-0011 把"常规运动"的仲裁权
交给本节点，其它写入者（MoveIt 轨迹、TUI 示教）只在**明确离权**时才动。

## 2. 分层（数学与 I/O 分开，数学层可桌面回归）

| 文件 | 职责 | 依赖 |
| --- | --- | --- |
| `node.py` | ROS I/O + 状态机（idle/nav/servo/hold）、安全闸门、发布 | rclpy / numpy |
| `model_builder.py` | 组合 URDF（底盘 + 臂）→ Pinocchio 模型与运动学 | pinocchio |
| `hqp.py` | 速度级 HQP：把任务栈化成 QP，OSQP 求解 | osqp / numpy |
| `joint_map.py` | 臂**两套关节命名与角度域**的单点映射 + 饱和（纯 stdlib） | 无 |
| `fake_arm_stream.py` | 软件臂夹具（仅测试/演示；不发真硬件） | rclpy |

**为什么 `joint_map` 单独存在**：运动学模型用 `rak_arm.urdf.xacro` 的名字
（`base_yaw_joint…`）与 **q_urdf** 域，而硬件/`joint_state_broadcaster` 用
`j1_base…` 与**舵机指令角**域，两者差一个零位偏移（J2 135°、J3 180°）。历史上节点
拿模型侧名字去查 `/joint_states`，**永远匹配不到**；发出去的又是 q_urdf 域。把
"名字 + 域 + 饱和"集中到一处，节点与测试共用（2026-10-28 修）。

## 3. 接口（与其它节点的契约）

- 订阅：`/cmd_vel`(Twist)、`/wbc/ee_target`(PoseStamped，带 frame)、`/wbc/mode`(String)、
  `/joint_states`(JointState)、TF(`map`→`base_link`)。
- 发布：底盘速度参考（`TwistStamped`，默认 `/mecanum_drive_controller/reference`）、
  臂位置流（`Float64MultiArray`，默认 `/wbc/arm_setpoints`，**指令角域** rad）、
  `/wbc/status`(String/JSON)。
- 服务：`/wbc/enable`(SetBool)。

接臂有两种写法（在 `rak_bringup` 里二选一，见 `chassis_bringup.launch.py` 的
`arm_motion_writer`）：`jtc` = MoveIt 的 `arm_controller`；`wbc` = 本节点 +
`arm_position_controller`（ForwardCommandController，`~/commands` 重映射到
`/wbc/arm_setpoints`）。**两者接口互斥，不能同时激活。**

## 4. 安全纪律（不可协商，见 `node.py` 头注）

- **默认 `enabled=false`**：不发任何运动命令；关闭时发一次零速后静默。
- 连续求解失败 ≥ `fail_limit` → `degraded`：停止发布，需显式重新 enable，**不自动恢复**。
- TF 丢失 → 发零速，不猜位姿。
- 不读 `/brain/decision`、不解析自然语言；目标必须带 frame。

## 5. 依赖与验证

- 运行期：`numpy` + `pinocchio`（PyPI `pin`）+ `osqp`。**rosdep 键未定**（见 package.xml
  注释与 ADR-0011 待闭合 #6）。
- 测试：`python3 -m pytest src/rak_wbc/test`（`conftest.py` 自带 sys.path，无需额外 PYTHONPATH）。
  **缺 pinocchio/osqp 的用例会自动 skip**（桌面无这两个库时：`12 passed, 16 skipped`）。
  在 Jetson 上应全跑（`test_hqp_numerical.py` / `test_model.py` / `test_joint_map.py` /
  `test_node_contract.py`）。
- 启动：`ros2 launch rak_wbc wbc.launch.py`（默认 `enabled=false`，另需 `publish_arm:=true`
  才发臂流；臂接口与实机几何仍是硬闸门）。

## 6. 仍未闭合（不要当已完成）

- `pinocchio`/`osqp` 在桌面缺席 → **HQP 数值层尚未在本机跑过**，上 Jetson 必须先全跑测试。
- 臂的真实质量/几何/关节限位仍是占位（`wbc_params.yaml` 里 `TODO(measured)`）。
- `publish_arm` 默认关；臂位置环（J3..J6 舵机 + J1/J2 编码器电机）的实机响应未验证。
