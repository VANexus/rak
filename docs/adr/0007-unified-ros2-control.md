# ADR-0007: 底盘与机械臂统一走 `ros2_control`，麦轮 IK 移到上位机

- **状态**: 已接受 —— **取代 [ADR-0001](0001-mecanum-kinematics-on-mcu.md)**；
  ⚠ 2026-10-09 起"USB 是唯一 Jetson↔MCU 链路"部分被 [ADR-0012](0012-transport-session-can-v4-primary.md) 取代
  （CAN v4 恢复为主链路，USB 冻结备用）；单 F407、主机侧 IK、EKF 航向等其余决策继续有效
- **日期**: 2026-10-05
- **相关**: `src/rak_description` · `src/rak_base_driver` · `src/rak_navigation` · `src/rak_moveit_config` · ADR-0002 · ADR-0003 · `vehical_rak_chassis/Core/Src/app_car.c`、`app_servo.c`

---

## 背景

### 硬件前提变了（2026-10-05）

| | ADR-0001 时（2026-10-04） | 现在 |
|---|---|---|
| MCU | **两块** F407（底盘 + 臂），CAN 互连 | **一块** F407（臂板烧毁），同时管 4 轮 + 6 舵机 |
| Jetson↔底盘 | UART 921600 | **USB**，唯一链路 |
| CAN | 底盘↔臂，classic 2.0A 1 Mbps | **已无用**（对端不存在） |
| IMU | 两块 F407 都带板载 IMU | **Jetson 的 LSM6DSRTR（SPI）做 EKF 融合**；F407 板载 IMU 只做本地麦轮自稳定 |

`vehical_rak_chassis/Core/Src/` 已含 `app_car.c`（麦轮 IK）、`app_servo.c`（6 路舵机 @50 Hz）、
`app_imu.c`（ICM20948 自稳定，R9–R14 迭代）——**一块板同时承担底盘与机械臂**。

### 问题

**ROS 侧的麦轮栈该怎么架构？** 原 ADR-0001 的结论（MCU 做 IK、Jetson 只发 `(vx,vy,wz)`）
在**两块 MCU + CAN** 的前提下成立。现在该前提消失：

- ADR-0001 理由 #2（"改 IK 归属会击穿 CAN v3 `0x100`"）—— **CAN 已无用，论据消失**。
- ADR-0001 理由 #4（"统一协议不成立，因为臂走 CAN 关节指令"）—— **现在臂和轮走同一条 USB**，
  统一协议反而成立且必要。
- 机械臂的 6 个舵机**必须**是关节级接口（MoveIt2 要 `JointTrajectoryController`）。
  若底盘是机体速度级、臂是关节级，**同一条链路上跑两套语义** —— 这正是要消除的。

---

## 决策

**全机走 `ros2_control`，统一关节级接口；麦轮逆运动学移到上位机。**

```
Nav2 ──/cmd_vel(Twist)──▶ mecanum_drive_controller ──┐
                                                      │ 4× wheel velocity cmd
MoveIt2 ──trajectory──▶ joint_trajectory_controller ──┤ 6× arm joint position cmd
                                                      ▼
                                        ┌──────────────────────────────┐
                                        │  rak_hardware (C++ plugin)   │
                                        │  hardware_interface::        │
                                        │         SystemInterface      │
                                        └──────────────┬───────────────┘
                                                       │ USB serial, COBS+CRC16
                                                       ▼
                                        ┌──────────────────────────────┐
                                        │  F407: 4×25 kHz 轮 PID        │
                                        │        + 6× 舵机 PWM @50 Hz   │
                                        │        + 编码器/关节反馈       │
                                        └──────────────────────────────┘
```

### D1. 底盘：`mecanum_drive_controller`（ros2_controllers 4.x，Jazzy 原生）

- 输入：`geometry_msgs/TwistStamped`（用 `linear.x`、`linear.y`、`angular.z`）
- 输出：**4 个轮关节的 `velocity` 命令接口**（rad/s）→ 硬件接口 → 串口 → F407
- 发布：`~/odometry`、`~/controller_state`
- 关键参数：`kinematics.wheels_radius`、`kinematics.sum_of_robot_center_projection_on_X_Y_axis`
  （= `wheel_base + wheel_separation`）、`reference_timeout`（**必须显式设**，见下）

**F407 不再做麦轮 IK**，只做：每轮 PID、舵机 PWM、编码器/关节反馈。

### D2. 机械臂：`joint_trajectory_controller`

6 个舵机走**位置**命令接口。MoveIt2 通过它下发轨迹。
（舵机是位置型执行器，不是力矩型——不要用 `effort` 接口。）

### D3. 统一硬件接口插件：`rak_hardware`

一个 `hardware_interface::SystemInterface` 插件同时暴露 4 轮 + 6 臂关节：

```xml
<ros2_control name="RakSystem" type="system">
  <hardware>
    <plugin>rak_hardware/RakSystemHardware</plugin>
    <param name="serial_port">/dev/wheel_motor</param>
    <param name="watchdog_ms">200</param>
  </hardware>
  <!-- 4 轮：velocity 命令 + velocity/position 状态 -->
  <joint name="wheel_fl_joint">
    <command_interface name="velocity"/>
    <state_interface name="velocity"/><state_interface name="position"/>
  </joint>
  <!-- ×4 ... -->
  <!-- 6 舵机：position 命令 + position 状态 -->
  <joint name="arm_joint_1">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
  <!-- ×6 ... -->
</ros2_control>
```

### D4. TF 与里程计的权威划分（避免两个发布者打架）

| 变换 | 唯一发布者 | 来源 |
|---|---|---|
| `odom → base_link` | **`robot_localization` EKF** | 轮式里程计（`mecanum_drive_controller/odometry`）+ IMU（LSM6DSRTR） |
| `map → odom` | **slam_toolbox** | 2D 雷达 |

→ **`mecanum_drive_controller` 必须设 `enable_odom_tf: false`**，
否则它与 EKF 抢发 `odom→base_link`。见 ADR-0002。

### D5. F407 板载 IMU：只做**轮级**稳定，不做机体航向（2026-10-05 用户确认）

- 板载 IMU 保留在 F407，用于**轮级**稳定/滑移补偿，是 `write()` 之下的一层内环。
- **机体航向权威是 Jetson 的 LSM6DSRTR**（EKF 融合）。
- 因为它是**轮级**而非机体级，**不与 Nav2 的 `wz` 指令冲突** —— 内环只影响单轮速度的执行品质，
  不改变机体速度语义。这是本轮确认的关键点，解除了上一稿的顾虑。
- 若将来该算法被改成机体级（例如自主做航向保持），**必须能与 Nav2 的 `wz` 互斥**，
  否则会互相打架。

### D6. 轮级饱和由 **MCU** 做（已核实控制器不提供）

**已核实**（`mecanum_drive_controller_parameters.hpp`，2026-10-05）：该控制器的限幅参数只有
`linear.x.*` / `linear.y.*` / `angular.z.*` —— **全是机体轴级，没有任何轮级饱和**。
即它**不做**"四轮等比缩放"。

原下位机 §10.2-1 的硬要求（三轮同时非零时某轮可能超限，**必须四轮同比例缩小，不得逐轴截断**）
因此**必须由 MCU 承担**：

> MCU 收到 4 个轮速目标后，若 `max(|v_i|) > V_LIMIT`，把**四个轮速同乘一个系数**。

**这在数学上等价于机体级等比缩放**：麦轮运动学是线性的，
四个轮速同乘 `k` ⟺ 机体速度 `(vx, vy, wz)` 同乘 `k`，**方向不变**。
所以把饱和放在轮级不损失正确性，且离执行器更近、更安全。

**双保险**：`linear.x/y`、`angular.z` 的限幅仍要设为保守的实测值（第一道防线），
但**不能依赖它来保证斜走方向** —— 保证方向的是 MCU 的等比饱和。

> 备查：Jazzy 里另有 `omni_wheel_drive_controller`（`libomni_wheel_drive_controller.so`），
> 那是给 **45° 全向轮**的，**不适用于麦轮**，不要选错。

---

## 理由

1. **统一关节级协议**。轮和臂现在同一条 USB，用同一套帧语义（命令+状态），
   而不是"轮用机体速度、臂用关节位置"两套。
2. **拿到 ros2_control 生态**：控制器生命周期、`joint_state_broadcaster`、
   速度/加速度限幅、`mecanum_drive_controller` 的标准里程计与协方差、
   `reference_timeout` 超时保护 —— 全部是经过测试的标准件，不用自建。
3. **仿真↔实机一键切换**。URDF 的 `<ros2_control>` 标签让 Gazebo 仿真与真机共用同一套控制器配置。
4. **固件反而变简单**：`app_car.c` 的麦轮 IK 从 MCU 移除（保留 PID、舵机、编码器）。
   少一段需要标定和维护的运动学代码。
5. **可调试性**：`ros2 control list_hardware_interfaces` / `list_controllers` 能看到每个关节的
   命令与状态，而不是一个不透明的 `(vx,vy,wz)` 黑洞。

---

## 后果

### 正面
- 全机一套控制器框架，Nav2 与 MoveIt2 用同一种方式接入硬件。
- 里程计由标准控制器产生，协方差可配置（对麦轮尤其重要，见 ADR-0002）。
- 未来换电机/加关节只改 URDF + 参数，不动协议。

### 负面（代价，必须承认）
- **需要改固件**：F407 的协议从"接收 `(vx,vy,wz)`"改为"接收 4 轮速度 + 6 关节位置"，
  并移除 `app_car.c` 的 IK。
  ✅ **2026-10-05 用户确认：固件仍在开发期、尚未成型** —— 所以这不是"推翻已验收成果"，
  而是**在成型前把接口改对**，成本远低于上一稿的估计。台架验收仍需按
  `hardware_acceptance.md` 重做（但那是本来就要做的）。
- **板载 IMU 算法不需降级**：确认它本来就是**轮级**稳定（D5），
  所以不存在"从机体级退让"的损失。保留 R9–R14 的成果。
- **轮级等比饱和仍由 MCU 承担**（D6）：已核实控制器不提供该功能，
  但轮级等比缩放**在数学上等价于**机体级等比缩放，因此 §10.2-1 的要求没有丢失，
  只是实现位置从"IK 之前"移到"IK 之后"。
- C++ 插件是新增的编译/维护面（`rak_gpu` 已有 C++ 约定可循）。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| **维持 ADR-0001**（MCU 做 IK，Jetson 只发机体速度） | 其成立前提（两块 MCU + CAN）已消失；同一条 USB 上跑两套协议语义；拿不到 ros2_control 生态 |
| 底盘用 Twist 直通、臂用 `joint_trajectory_controller` | 同一条链路上两套语义，硬件接口要分叉，正是本次要消除的 |
| 给底盘造一个自定义 "twist 命令接口" | ros2_control 没有标准的 twist 命令接口；等于绕开生态自建一套 |
| 继续用 `rak_base_driver` 的 `(vx,vy,wz)` 串口协议 | 与关节级接口并存会导致重复的限幅/超时/里程计实现 |

---

## 待闭合项

1. **固件协议重写**：4 轮速度 + 6 关节位置的下行帧，4 轮速度/位置 + 6 关节位置的上行帧。
   建议沿用已实现的 COBS + CRC16 + 序号 + lease 看门狗（见 ADR-0003）。
2. ✅ **已核实并解决（2026-10-05）**：`mecanum_drive_controller` **只有机体轴级限幅，
   不做轮级饱和**。→ 轮级等比饱和改由 MCU 承担（见 D6），数学上等价，斜走方向不丢。
   落地时要验证：给定 `vx=vy=wz` 同时非零且超限时，四轮确实同比例缩小、方向不变。
3. **`enable_odom_tf: false`** + `reference_timeout` 必须显式配置
   （`reference_timeout` 默认 0 会导致"每周期重置参考"，在链式模式下有已知危险行为）。
4. **`mecanum_drive_controller` 的已知局限**：不做滑移补偿、无辊子角参数（假定理想 45°）、
   里程计**刻意不做滤波** —— 所以必须靠 ADR-0002 的 EKF 融合 IMU。
5. **舵机映射未知**：6 路舵机的关节名、零位、方向、限位、PWM 脉宽范围需与固件对齐。
6. **板载 IMU 自稳定算法的处置**：是降为轮级、还是仅在手动模式启用，需明确定义。
7. **`rak_base_driver` 的归宿**：被 `rak_hardware` 取代后，其 codec/PTY 测试资产应迁移还是废弃。
8. **`rak_can_driver` / `rak_serial_link` 的归宿**：CAN 已无用，应明确标记为废弃或保留待用。

---

## 参考

- `mecanum_drive_controller` Jazzy 用户文档：<https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html>
- ros2_control 硬件组件编写：<https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html>
- `robot_localization` 配置（Nav2 官方）：<https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/odom/setup_robot_localization/>
- `vehical_rak_chassis/Core/Src/app_car.c`、`app_servo.c`、`app_imu.c`（现固件能力）
- ADR-0001（被本篇取代）、ADR-0002（EKF 与 TF 权威）、ADR-0003（协议归属）
- 结论部分来自 2026-10-05 麦轮架构专项研究
