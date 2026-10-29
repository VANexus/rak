# 机械臂 MoveIt 冻结契约（CAN v4 之上的唯一稳定接口）

> 状态：**FROZEN**（2026-10-09，随 CAN v4 协议冻结）。MoveIt/控制器配置只能消费本契约，不得要求改动协议或 Session API。
> 下游协议事实源：`vehical_rak_chassis/docs/PROTOCOL.md` v4 §5（JxANG/GRIP 量化）；本文件只定义 MoveIt 侧如何消费。

## 1. 关节定义（URDF `<ros2_control>` 与本契约一一对应）

| 序号 | URDF joint 名 | 协议轴 | 物理通道 | 行程 | Home 位 |
| --- | --- | --- | --- | --- | --- |
| 0 | `j1_base`（腰） | J1 | **A2 起 = MG513P30 增量编码器电机**（AT8236 双输入 + 正交编码器；占位=电机 A / TIM2，待实物插头确认） | 0…270°（待标定） | 0° |
| 1 | `j2_shoulder`（大臂） | J2 | **A2 起 = MG513P30 增量编码器电机**（AT8236 + 正交编码器；占位=电机 B / TIM3，待确认） | 0…270°（固件 J2 上限 255°） | 225° |
| 2 | `j3_elbow`（肘） | J3 | ch3 (PC9) | 0…270° | 270° |
| 3 | `j4_forearm`（小臂） | J4 | ch2 (PC8) | 0…270° | 0° |
| 4 | `j5_wrist_pitch`（腕俯仰） | J5 | ch1 (PC7) | 0…270° | 0° |
| 5 | `j6_wrist_roll`（腕圆周） | J6 | ch0 (PC6) | 0…270° | 0° |
| 6 | `gripper_joint`（夹爪） | GRIP | ——无硬件通道 | —— | —— |

- 单位一律 **rad**（URDF/ros2_control 惯例）；Session API `set_arm_command(array<double,7>)` 收 rad，内部按协议量化 0.1°。
- 上限：[0, 4.71239 rad]（0…270°）。越界由 Session 层饱和截断（负值截到 0，协议禁止回绕）。
- ⚠️ **J1/J2 从舵机改为编码器电机（2026-10-28，臂板固件 `A2`），协议与契约不变**：
  仍是 0.1° 目标 / 实际角回读；只是执行机构换了。**J1/J2 增量编码器掉电丢零点**——
  每次冷启动要在臂板上人工对位 + 长按用户键 PE0 ≥1.5 s 软件置零（见臂板
  `docs/ARM-ENCODER-JOINTS-DESIGN.md` §5）；**未归零时 J1/J2 恒刹车、不接受指令**。
  设计/精度/50 kHz 论证见该文档 §7–§8。

## 2. 命令与反馈语义（MoveIt 控制器设计依据）

1. **命令接口**：`position`（位置控制）。轨迹点 → `set_arm_command`，事件驱动发送（CanSession 仅在 `motion_enabled` 且目标变化时发 A/B 组，≤50 Hz）；MoveIt 侧**必须先调 `set_motion_enabled(true)`**（安全门，见 PROTOCOL §10.1 #7：固件对舵机不做门控，许可在上位机）。
2. **反馈接口**：`position` 读自 `SessionSnapshot.joint_position[0..5]`，`servo_valid_mask` 标出有效位。**反馈精度分两类（A2 起）：**
   - **bit0/bit1（J1/J2）= 真测量**：增量编码器回读，分辨率 ≈0.0059°/计数（电机输出轴），
     好于协议量化 0.1° 一个数量级。但**减速箱回差（~0.5–2°）才是精度瓶颈**，且编码器在
     减速箱**前** → 回差无法被闭环消掉。**J1/J2 可用真反馈做到位判定**（容差别小于回差量级）。
   - **bit2..5（J3..J6）= 舵机脉宽回声，不是物理测量**。MoveIt 的轨迹完成判定**不得**依赖
     其精度优于指令分辨率（0.1°），**碰撞/卡死检测不可用**该反馈。bit6（夹爪）恒无效。
3. **夹爪（第 7 轴）**：命令通道协议已预留（GRIP_u16），**回读恒 0 且 `servo_valid_mask` bit6 恒无效**（实测契约，`can_full_motion_test` 验证）。硬件接入前 MoveIt 配置将其设为不可控伪关节或固定关节。
4. **速率**：CAN ARM 指令 ≤50 Hz；舵机物理响应（PWM 50 Hz）远慢于指令率——轨迹下发的平滑由 MoveIt TOTG/速度标量负责，Session 层不插值。
5. **安全**：急停作用于轮驱动 **+ J1/J2 电机**（`joint_set_estop`：短路制动 + 锁存，≤40 µs 生效）；
   **J3..J6 舵机不受急停影响**（保持当前位置）。上电固件自动输出 Home 位（R44）。
   MoveIt 启动顺序：Session start → 等 `handshaken` → `set_motion_enabled(true)` → 才允许发轨迹。
   ⚠️ **J1/J2 未软件归零时恒刹车、不接受任何目标**（与 `motion_enabled` 无关，归零在臂板本地完成）。

## 3. 验证基线（2026-10-09，R63 固件）

- 全行程逐轴扫描（0→90→180→270→90→0，±2° 回读一致）+ 夹爪契约 + 回 Home：`scripts/can_full_motion_test.cpp` 全过。
- 编译：`g++ -std=c++17 -I install/rak_chassis/include -I src/rak_chassis/include <test>.cpp src/rak_chassis/src/can_session.cpp -o <out>`（零 ROS 依赖）。
