# ADR-0001: 麦轮逆运动学放在 F407，上位机只下发 `(vx, vy, wz)`

- **状态**: **已被 [ADR-0007](0007-unified-ros2-control.md) 取代（2026-10-05）**
- **日期**: 2026-10-05（决策实际确立于 2026-10-04 包迁移前后，本文补记理由与待闭合项）
- **取代原因**: 硬件前提变更。臂板烧毁后只剩**一块** F407（同时管 4 轮 + 6 舵机），
  **CAN 链路消失**，Jetson 只通过 USB 与下位机通信。
  本文的两条核心论据 ——「改 IK 归属会击穿 CAN v3 `0x100`」与
  「统一协议不成立（臂走 CAN 关节指令）」—— 随之失效。
  **保留本文以记录当时的推理与被否决的方案，不要按本文实施。**
- **相关**: `docs/explanation/architecture.md` §1/§4 · `docs/reference/hardware_acceptance.md` · `docs/protocol/jetson_f407_uart_v1.1.md` · `src/rak_base_driver` · `src/rak_motion_stub` · `src/rak_navigation/config/nav2_params.yaml`

---

## 背景

硬件事实（来源：`docs/explanation/architecture.md` §1、`docs/reference/hardware_acceptance.md` §Known Topology）：

- 底盘是 **4 轮麦轮**，F407 自己跑**四轮 25 kHz 闭环**。
- **Jetson 不直连 CAN**。CAN 只存在于 F407↔F407 之间（classic CAN 2.0A，1 Mbps，1 kHz）。
- Jetson↔底盘 F407 是 **USB-UART，921600 baud，100 Hz 双向**。
- 上位机 `(vx, vy, wz)` 由 Nav2 产生；底盘回传机体速度遥测。

要决定的问题是：**麦轮逆运动学（`(vx, vy, wz)` → 4 个轮速）放在哪一侧算？**

---

## 决策

**逆运动学放在 F407。上位机只下发机体速度 `(vx, vy, wz)`，永不下发轮速。**

落实到三处，缺一不可：

1. **协议层**：`SET_BASE_VELOCITY` v1.1 载荷 19 字节，含 `vx_mm_s` / `vy_mm_s` / `wz_mrad_s`
   （i32 LE）+ `action_type` + `max_exec_ms`。见 `docs/protocol/jetson_f407_uart_v1.1.md`。
2. **Nav2 层**：`nav2_mppi_controller::MPPIController` + `motion_model: "Omni"`，
   `vy_max: 0.15`（`nav2_params.yaml:50-64`）。**全向模型是前提**，否则 vy 恒为 0。
3. **ROS 层禁用 `mecanum_drive_controller`**。该组件的作用恰好是把 IK 放回 Jetson，
   与本决策互斥；引入它等于同一套运动学在两处实现。

---

## 理由

1. **串口带宽**。下发 3 个 i32（19 B/帧 @ 100 Hz ≈ 1.9 kB/s），而不是 4 轮速 + 序号 +
   时间戳。串口是本系统最紧张的链路，尤其它还要承载 vy。
2. **实时性**。25 kHz 轮环必须留在 MCU。若上位机算轮速，任何一次串口抖动或 Linux 调度
   延迟都会直接变成轮速阶跃；下发机体速度时，MCU 可以用自己的插值/限幅吸收抖动。
3. **安全语义更干净**。断联时 MCU 持有的是"期望机体速度"，可以直接衰减到零，**不需要知道
   上位机的 IK 参数**（轮径、轮距、辊子角）。`hardware_acceptance.md` 要求 MCU 独立实现
   命令租约、超时、E-stop、故障锁存 —— IK 在 MCU 上时这些都不依赖上位机。
4. **标定归属正确**。轮径/轮距/辊子角误差属于底盘机械参数；IK 在 MCU 上时，改这些参数
   不需要重新部署 ROS 侧。
5. **与拓扑一致**。Jetson 根本不在 CAN 总线上，物理上就无法做轮级控制。

---

## 后果

### 正面

- 上位机与底盘机械解耦：换轮径、调轮距不触碰 ROS 侧。
- 串口带宽与抖动裕度更大，为后续加遥测字段留了空间。
- MCU 能独立实现命令租约/超时/急停，符合硬件验收要求。

### 负面（代价，必须承认）

- **上位机拿不到轮级反馈**，只有机体速度遥测。轮级故障（单轮堵转、编码器损坏）在上位机
  侧表现为"里程计漂移"，根因定位困难。**这是本决策最主要的技术债。**
- **麦轮里程计在 y 方向本身很差**（辊子侧滑、地面摩擦各向异性）。由 `(vx, vy, wz)` 积分出的
  位姿，横移分量不可信。→ 必须靠 IMU/雷达/视觉融合补偿，见 ADR-0002。
- **失去 ros2_control 控制器生态**。用不了 `mecanum_drive_controller`，速度限幅、平滑、
  里程计发布、`/cmd_vel` 单写者仲裁都要自己做（现在由 `cmd_vel_stub` 和
  `nav2` 的 smoother/collision-monitor 承担，见 `docs/explanation/architecture.md` §4）。
- 上位机侧无法做"四轮转速一致性检查"这类诊断。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| `mecanum_drive_controller` 在 Jetson 算 4 轮转速，串口下发轮速 | 见下方"反方论证"——这是**唯一有真实分量的反方**，本决策的成立依赖固件与协议耦合，不是靠带宽/实时性 |
| Jetson 直连 CAN 做轮级控制 | 硬件拓扑不支持（Jetson 不在 CAN 总线）；会把实时闭环搬回非实时主机 |
| 上位机算 IK、MCU 再算一遍做交叉校验 | 两处实现同一套运动学，标定漂移与版本漂移风险翻倍，收益不明确 |

### 反方论证（2026-10-05 外部研究提出，必须记录）

独立研究（ROS 2 社区实践口径）**建议相反方案**：IK 放 Jetson 的 `mecanum_drive_controller`，
串口下发 4 个轮速（rad/s），MCU 只跑 PID/编码器。其论据：

- **带宽不是理由**：两种方案都是 ~16 B/帧，50 Hz 下不到 921600 UART 的 2%，更别说 USB CDC。
  → **本文原"理由 1（串口带宽）"因此被削弱，不应再作为主要论据。**
- ros2_control 的控制器链**白送**速度限幅、超时、里程计发布、TF，且是经过测试的标准件。
- 底盘与机械臂可共用一套统一协议："下发轮速目标、回传轮速+位置"。
- `mecanum_drive_controller` 的里程计由轮状态反馈算出，是**标准且可调试**的路径。

**为什么本决策仍然成立**（真正的理由，已从上表中替换）：

1. **固件已实现并台架实测**。`app_mecanum` 做 IK + §10.2-1 的**矢量等比饱和**，
   R16–R18 已在实车跑过。改方案 = 重写固件 + 重做台架验收。
2. **协议耦合是双向的**。UART v1 载荷是 `vx/vy/wz`；**CAN v3 `CHASSIS_VEL_CMD`(0x100) 同样是
   Vx/Vy/ωz**，而底盘↔臂的 `XFER_ARM_TO_CH`(0x031) 也建立在这套语义上。
   改 IK 归属会同时击穿 UART v1、CAN v3 与臂侧协议。
3. **MCU 侧等比饱和是硬要求**（§10.2-1：斜走时四轮必须同比例缩放，不得逐轴截断）。
   若 IK 移到 Jetson，这条约束要跨进程重新实现，且失去 MCU 的独立保护。
4. **"统一协议"在本项目不成立**：臂走的是 CAN 关节指令（0x101/0x102），
   与底盘的 UART 机体速度本就是两套东西，统一协议带来的收益在这里是空的。

**代价（必须承认）**：放弃 ros2_control 生态，速度限幅/超时/里程计/TF 全部要自建或由
EKF 补（ADR-0002 已覆盖里程计与 TF；限幅现由 `max_vx/vy/wz_m_s` 参数承担）。

**触发重新评估的条件**：若底盘固件需要推倒重写，或 CAN v3 协议需要大改，
应重新权衡本决策 —— 那时切换的边际成本最低。

---

## 待闭合项

1. **vy 链路的真实阻塞点是配置，不是代码（2026-10-05 核对更正）。**
   逐环核对结果：`cmd_vel_stub._cb()` **发布整个 Twist**（`self.raw_pub_.publish(msg)`），
   `rak_base_driver/node.py:192` 读的是 `(msg.linear.x, msg.linear.y, msg.angular.z)`，
   `protocol.py` 的 **v1（15 B）与 v1.1（19 B）都带 `vy_mm_s`**。
   即 **ROS 侧 vy 通路是通的**，过期的是文档（`cmd_vel_stub` docstring 自称
   "unicycle contract is the only contract"、日志只打 `lin.x`/`ang.z`）。
   **真正的阻塞是 `node.py:71` 的 `max_vy_m_s` 默认 `0.0`** —— 驱动要求三个限值都是
   有限正数才允许 ARM，所以 vy 被限成 0 且无法 ARM。
   → 待办：填入实测 vy 限值；同步 `cmd_vel_stub` 的注释/日志为"全向合同"。
   （另：`vehical_rak_chassis/docs/PROTOCOL.md` §10.2-2 仍写"Nav2 必须换
   `nav2_mecanum_controller`"—— 该表述已过期，`nav2_params.yaml` 用
   `nav2_mppi_controller` + `motion_model: "Omni"` 已达成同一效果。）
2. **`cmd_vel_stub` 的定位要澄清**：它现在同时承担"转发"与"仿真遥测估算"（`cmd_vel_sim.py`），
   命名与文档已过期。
3. **速度限值是占位值**：`vx 0.25 / vy 0.15 / wz 0.5`（`nav2_params.yaml:171-172` 自注
   "simulation-only; replace from measured chassis limits"）。实车限速必须实测。
4. **collision monitor 当前"不介入"**（`CLAUDE.md`）。开了 Omni 之后，横移路径的安全边界
   需要重新评估 —— 麦轮横移时车体扫过的面积与前进不同。
5. **上位机侧底盘诊断缺失**：没有轮级反馈，需要定义"里程计可信度"如何上报（协方差？
   还是显式 fault 位？）。
6. **UART 物理层与本仓库文档不一致**：`hardware_acceptance.md` 写 USB-UART 且 bringup 用
   `/dev/wheel_motor` 别名，但 2026-10-05 在 Jetson 上额外配了 40-pin 的
   `/dev/robot_uart`（`ttyTHS1`，见 `scripts/lsm6dsr-overlay/`）。需确认底盘实际接哪个口，
   并把 udev 别名统一到 `/dev/wheel_motor`。

---

## 参考

- `docs/explanation/architecture.md` §1（硬件事实）、§4（执行侧）
- `docs/reference/hardware_acceptance.md` §Known Topology、§Before Connecting a Physical Base
- `docs/protocol/jetson_f407_uart_v1.1.md`（`SET_BASE_VELOCITY` 19 字节载荷与 `action_type` 表）
- `src/rak_navigation/config/nav2_params.yaml:50-64, 171-172`
- `src/rak_motion_stub/rak_motion_stub/cmd_vel_stub.py`（待更新的过期文档字符串）
