# 机械臂 MoveIt 冻结契约（CAN v4 之上的唯一稳定接口）

> 状态：**FROZEN**（2026-10-09，随 CAN v4 协议冻结）。MoveIt/控制器配置只能消费本契约，不得要求改动协议或 Session API。
> 下游协议事实源：`vehical_rak_chassis/docs/PROTOCOL.md` v4 §5（JxANG/GRIP 量化）；本文件只定义 MoveIt 侧如何消费。

## 1. 关节定义（URDF `<ros2_control>` 与本契约一一对应）

| 序号 | URDF joint 名 | 协议轴 | 物理通道 | 行程 | Home 位 |
| --- | --- | --- | --- | --- | --- |
| 0 | `j1_base`（腰） | J1 | ch5 (PB15) | 0…270° | 0° |
| 1 | `j2_shoulder`（大臂） | J2 | ch4 (PB14) | 0…270° | 225° |
| 2 | `j3_elbow`（肘） | J3 | ch3 (PC9) | 0…270° | 270° |
| 3 | `j4_forearm`（小臂） | J4 | ch2 (PC8) | 0…270° | 0° |
| 4 | `j5_wrist_pitch`（腕俯仰） | J5 | ch1 (PC7) | 0…270° | 0° |
| 5 | `j6_wrist_roll`（腕圆周） | J6 | ch0 (PC6) | 0…270° | 0° |
| 6 | `gripper_joint`（夹爪） | GRIP | ——无硬件通道 | —— | —— |

- 单位一律 **rad**（URDF/ros2_control 惯例）；Session API `set_arm_command(array<double,7>)` 收 rad，内部按协议量化 0.1°。
- 上限：[0, 4.71239 rad]（0…270°）。越界由 Session 层饱和截断（负值截到 0，协议禁止回绕）。

## 2. 命令与反馈语义（MoveIt 控制器设计依据）

1. **命令接口**：`position`（位置控制）。轨迹点 → `set_arm_command`，事件驱动发送（CanSession 仅在 `motion_enabled` 且目标变化时发 A/B 组，≤50 Hz）；MoveIt 侧**必须先调 `set_motion_enabled(true)`**（安全门，见 PROTOCOL §10.1 #7：固件对舵机不做门控，许可在上位机）。
2. **反馈接口**：`position` 读自 `SessionSnapshot.joint_position[0..5]`，`servo_valid_mask` bit0-5 有效。⚠️ **反馈=固件指令回读（回声），不是物理测量**——舵机无编码器。MoveIt 的轨迹完成判定不得依赖反馈精度优于指令分辨率（0.1°），碰撞/卡死检测不可用该反馈。
3. **夹爪（第 7 轴）**：命令通道协议已预留（GRIP_u16），**回读恒 0 且 `servo_valid_mask` bit6 恒无效**（实测契约，`can_full_motion_test` 验证）。硬件接入前 MoveIt 配置将其设为不可控伪关节或固定关节。
4. **速率**：CAN ARM 指令 ≤50 Hz；舵机物理响应（PWM 50 Hz）远慢于指令率——轨迹下发的平滑由 MoveIt TOTG/速度标量负责，Session 层不插值。
5. **安全**：急停只作用于轮驱动，**舵机不受急停影响**（保持当前位置）；上电固件自动输出 Home 位（R44）。MoveIt 启动顺序：Session start → 等 `handshaken` → `set_motion_enabled(true)` → 才允许发轨迹。

## 3. 验证基线（2026-10-09，R63 固件）

- 全行程逐轴扫描（0→90→180→270→90→0，±2° 回读一致）+ 夹爪契约 + 回 Home：`scripts/can_full_motion_test.cpp` 全过。
- 编译：`g++ -std=c++17 -I install/rak_chassis/include -I src/rak_chassis/include <test>.cpp src/rak_chassis/src/can_session.cpp -o <out>`（零 ROS 依赖）。
