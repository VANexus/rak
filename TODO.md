# TODO — 感知 / 融合 / 抓取 路线图

- **日期**: 2026-10-05
- **状态**: 规划中（未开始实施）
- **配套**: 决策理由见 [`docs/adr/`](docs/adr/README.md) · 装配顺序见 [`docs/CONSTRUCTION.md`](docs/CONSTRUCTION.md) ·
  系统结构见 [`docs/architecture.md`](docs/architecture.md) · 上车门槛见 [`docs/hardware_acceptance.md`](docs/hardware_acceptance.md)
- **⚠️ 本文是 2026-10-05 的路线图，已被 2026-10-29 的一轮工程化部分覆盖。**
  **当前正在执行的施工单是 [`docs/NEXT-2026-10-30.md`](docs/NEXT-2026-10-30.md)**
  （组装层拆分 / 生命周期 / 时间基准 / 观测 / CANopen v5 / 行为层），
  两者冲突时以 ADR（0020/0023/0024/0025/0026）与施工单为准。

---

## 怎么读这份文档

- **P0 不解决，后面全是空转。** P0 里有一半是"已实现的代码被配置卡住"，不是缺功能。
- 每项格式：**目标 / 前置 / 验收 / 风险**。验收必须可执行，不能写"工作正常"。
- `[已核实]` = 2026-10-05 直接查代码/板子得到；`[待核实]` = 来自文档声明，未验证；
  `[需实测]` = 占位值，必须测量替换。
- **本文只列任务，不写决策理由。** 理由在 ADR 里，避免两处漂移。

---

## 优先级总览

| 级别 | 主题 | 阻塞谁 | 关键前置 |
|---|---|---|---|
| **P0** | 底盘链路可用（vy / lease / 协议裁定 / 实车验收） | 全部运动功能 | 下位机签字 |
| **P0** | IMU（LSM6DSRTR）接入 | 里程计融合、航向 | 接线 + overlay |
| **P1** | 四路里程计融合 + 3D 避障 | 自主导航质量 | P0 完成 |
| **P2** | 感知算力分配 + 深度点云 + 视觉点云融合 | 抓取、语义避障 | P1 部分 |
| **P2** | Harness 慢路径 + 呼吸感表达层（P2-7，ADR-0010） | 任务理解、对话 | 无硬件前置（纯软件） |
| **P3** | 抓取（几何 → 网络 → VLM） | 机械臂作业 | P2 + 臂实机解锁 |

---

## 分工门类索引（2026-10-11 整理，供分工，不改任务内容）

按"谁来干"归类（门类 = 技能/责任域）。**门禁**列区分：`硬` = 需实物/下位机才能闭环；
`软` = 无硬件前置，纯软件可独立推进；`裁` = 先出裁定/结论，不写代码。
本表只做索引，任务定义与验收仍在各自 P-项里。

| 门类 | 负责域 | 条目 | 门禁 |
|---|---|---|---|
| **A 下位机 / 固件**（下位仓协作） | F407 固件、协议 | P0-1③（下行改 4 轮速度+6 关节位置、移除 `app_car.c` 麦轮 IK）、P0-2（lease 200 ms 签字）、P0-4 前置（build_tag/协议/feature bits 确认） | 硬 |
| **B 上位机运动 / 底盘链路** | `ros2_control`、传输、实车 | P0-1（`rak_chassis` 插件、URDF 标签、spawner、`enable_odom_tf:false`）、P0-4（CAN 实车六步验收上位侧）、P1-5（WBC/HQP 上车） | 硬 |
| **C 传感器 / 内核集成** | 接线、overlay、驱动 | P0-5（IMU 接线/overlay/IIO→`sensor_msgs/Imu` 桥）、P3-5（末端相机选型+内参+手眼标定） | 硬（IMU 桥节点为软） |
| **D 状态估计 / SLAM / 导航** | EKF、rf2o、Nav2、坐标系 | P1-1（四路融合 F1→F3）、P1-2（`odom_source:=ekf` 真实现）、P1-3（STVL 3D 避障）、P1-4（融合 SLAM）、P1-6（坐标系数值回填+REP-120 复测） | 混合（P1-2 为软） |
| **E 感知栈 / 算力** | 点云、检测、算力裁定 | P2-1（算力分配裁定）、P2-2（DeepStream/Holoscan 结论+样例）、P2-3（深度点云管线）、P2-4（视觉+点云融合→`TrackedObjects`）、P2-5（超远物体建模） | 裁 + 硬 |
| **F 决策 / 任务 / 交互软件** | Agent、任务、语音、Harness | P2-6（ASR/TTS 接入）、P2-7（Harness + 呼吸感表达层，ADR-0010） | 软 |
| **G 机械臂 / 抓取** | MoveIt、抓取算法、臂实机 | P3-1（点云几何抓取——**本次已整合离线 GraspNet 链**）、P3-2（抓取网络 GraspNet-1B）、P3-3（VLM 抓取）、P3-4（臂 ros2_control 适配）、P3-5（手眼标定）、P3-6（URDF 重建+零位精标+MoveIt 规划配置） | G 前段软、P3-4 及零位精标硬 |
| **H 主机/环境（非仓库代码）** | 板子配置 | 附：本机变更（SSH、zram+swap、nvpmodel、udev 别名）——换板/重装需重做 | 硬 |

**门禁摘要**：现在**无硬件前置、可独立开工**的门类 = **F（全部）** 与 **G 前段（P3-1/2/3、P3-6 的 MoveIt 规划配置与 URDF）**、以及 **D 的 P1-2**；
其余门类（A/B/C/E 大半/H）**卡在下位机固件、实物接线或算力裁定**，先做裁定或等硬件。

---

## P0 — 阻塞项

### P0-1 迁移到统一 `ros2_control` 架构（当前最大任务）

- **目标**：全机改走 `ros2_control` —— 底盘 `mecanum_drive_controller`、
  机械臂 `joint_trajectory_controller`、一个 `hardware_interface::SystemInterface` 插件走 CAN v4。
- **依据**：**ADR-0007（取代 ADR-0001）**。硬件前提已变：只剩一块 F407 同时管 4 轮 + 6 舵机
 （2026-10-14 起 Jetson↔F407 走 CAN v4 唯一链路，ADR-0016）。
- **要做**：① 写 `rak_chassis` C++ 插件（4 轮 velocity + 6 关节 position）；
  ② URDF 加 `<ros2_control>` 标签；③ 固件协议改为"4 轮速度 + 6 关节位置"下行、
  "4 轮速度/位置 + 6 关节位置"上行（**移除 `app_car.c` 的麦轮 IK**）；
  ④ 配 `controller_manager` + spawner；⑤ `enable_odom_tf: false` + 显式 `reference_timeout`。
- **验收**：`ros2 control list_hardware_interfaces` 能看到全部接口；
  纯 vy 指令下四轮按麦轮关系反转（对角同向、相邻反向）；斜走方向不偏。
- **已解决的关键疑点** `[已核实 2026-10-05]`：`mecanum_drive_controller` **只有机体轴级限幅，
  不做轮级饱和**（`linear.x/y`、`angular.z`）。→ 轮级等比饱和**由 MCU 承担**：
  四个轮速同乘一个系数即可，因运动学线性，等价于机体速度等比缩放，**斜走方向不丢**。
- **风险**：固件仍在开发期（用户确认未成型），所以改接口是"成型前改对"，成本低；
  但仍须按 `hardware_acceptance.md` 做台架验收（本来就要做）。
- **关联**：ADR-0007、ADR-0002、ADR-0003

### P0-2 lease 时长分歧签字

- **目标**：上位机与下位机对命令租约达成一致并落地。
- **现状** `[已核实]`：规范写 `[10, 50] ms`，下位机实发 **200 ms**
  （`vehical_rak_chassis/docs/UART-v1-ALIGN.md`：电机 PWM 运行时 USART ORE 丢字节，
  50 ms 必触发超时锁存；IRQ 优先级 7→4 后 ORE 降 84%）。
- **前置**：团队批准 200 ms 作为台架/实车过渡值。
- **验收**：上位机按协议发 200 ms lease，实车行驶 5 min 无 lease 超时锁存；断开链路后 MCU 在
  lease 内独立停止。（原验收载体 `rak_base_driver/node.py` 已随 ADR-0016 删除，需以 CAN 侧
  实现重做。）
- **风险**：200 ms 意味着**断联后底盘最多再走 200 ms** —— 高速下的滑行距离必须核算。
- **关联**：ADR-0003 §A1

### P0-3 ~~裁定"Jetson 是否在 CAN 总线上"~~ ✅ 已裁定

- **结论（2026-10-05 用户确认）**：Jetson 只通过 USB 与下位机通信——**已被 ADR-0012
  （2026-10-09）取代：CAN v4 恢复主链路；再被 ADR-0016（2026-10-14）取代：USB v2
  链路删除，CAN v4 是唯一传输**，`Session` 抽象切换。
- **遗留动作 [2026-10-09 完成]**：`rak_can_driver` 与 `rak_serial_link`（CAN v3 host 栈）
  已删除（2026-10-10 用户裁定，原 `attic/` 本地归档不再保留；代码仅存 git 历史）；
  活 CAN 路径 = `src/rak_chassis` 的 `CanSession`。
- **关联**：ADR-0003 §A3（已更新）、ADR-0007

### P0-4 底盘链路实车验收（CAN v4）

- **目标**：从"launch 能解析"走到"真实 F407 上跑通"。
- **前置**：下位机固件确认（build_tag / 协议版本 / feature bits）。
- **验收**：严格按 [`hardware_acceptance.md`](hardware_acceptance.md) §Staged Base Test 六步：
  链路冒烟（`python3 scripts/can_link_smoke.py can0`，10/10 通过）→ 只读遥测 →
  拔链路/超时/E-stop 撤销 → 轮子离地单轴低速 → 软件暂停与物理急停分开 →
  围栏地面测试。
- **风险**：上位机文档仍写"仓库没有匹配的 F407 固件"，**该声明已过期**（ADR-0003 §A5），
  会误导验收准备。先更正文档。
- **关联**：ADR-0003 §A5、ADR-0016、`docs/CONSTRUCTION.md` §6

### P0-5 IMU（LSM6DSRTR）接入

- **目标**：Jetson 侧 IMU 可用，作为**航向权威源**。
- **依据** `[已核实]`：下位机 §10.1 决定 #2 —— **"航向权威源 = Jetson 的 LSM6DSRTR（SPI）"**，
  两块 F407 不读板载 ICM20948，MCU 不做姿态/航向，CAN 帧里不加 yaw 字段。
  → **IMU 不是可选项，是已定的架构组件。**
- **现状** `[已核实]`：overlay 已备好并通过 `dtc` + `fdtoverlay` 验证，**未接线、未应用**。
- **步骤**：① 接线（SPI 19/21/23/24，DRDY→pin15）→ ② `scripts/lsm6dsr-overlay/apply.sh` →
  ③ reboot → ④ `verify.sh` 确认 `spi0.0 -> st_lsm6dsx_spi` → ⑤ **实现 IIO→`sensor_msgs/Imu` 桥接节点 [待实现]**。
- **验收**：静止时 gyro 零偏可测；绕 z 轴旋转 360° 积分误差 < 阈值；`/proc/interrupts` 有中断计数。
- **风险**：IIO 桥接节点的**时间戳来源**必须用内核采样时间戳，不能用"处理完的时间"
  （`docs/architecture.md` §5 明确禁止）。
- **关联**：`docs/CONSTRUCTION.md` §5、ADR-0002

---

## P1 — 融合 SLAM 与避障

### P1-1 四路里程计融合

- **目标**：轮式 + IMU + 雷达（rf2o）+ 视觉 四路融合，输出可信的 `odom→base_footprint`（REP-120，ADR-0015）。
- **前置**：P0-4（轮式遥测）、P0-5（IMU）。
- **分阶段**（ADR-0002）：F1 = 轮式+IMU → F2 = +雷达 → F3 = +视觉，每阶段实测通过再进下一步。
- **验收**：横移 1 m 后 y 方向漂移显著小于纯轮式；静止 60 s 无漂移；长直走廊不退化。
- **风险**：**麦轮里程计 y 方向精度差是麦轮的固有代价**（辊子滑移对编码器不可见），
  与 IK 放在哪一侧无关；融合是唯一出路（见 ADR-0002/0007）；
  三者的坐标系/符号对齐是最易错且最难排查的一步。

### P1-2 实现 `odom_source:=ekf` 分支

- **目标**：把预留接缝变成真实现。
- **现状** `[已核实]`：`docs/architecture.md` §7 明确"`odom_source:=ekf` 预留选项
  **不会自动启动 EKF**"。rf2o 有 `publish_tf` 参数（默认 true），接管时须置 false，
  否则**两个节点抢发 `odom→base_footprint`**。
- **验收**：`ros2 launch rak_bringup slam.launch.py odom_source:=ekf` 后
  `ros2 run tf2_tools view_frames` 只有一个 `odom→base_footprint` 发布者。

### P1-3 3D 避障

- **目标**：在 2D 代价地图之上加 3D 障碍层，覆盖低矮/悬空障碍。
- **前置**：深度相机点云可用（P2-3）。
- **已定**（ADR-0006）：**STVL**（`spatio_temporal_voxel_layer`，纯 CPU，不抢显存）。
  `nvblox_nav2` 仅作备选；`voxel_layer` 已否决（CPU 占用约为 STVL 两倍）。
- **验收**：桌腿、悬空横杆能被观测到并让 Nav2 规划绕行。
- **风险**：4 GB 内存下 3D 体素层是**主要内存消耗者**；可能必须与感知栈分时复用（见 P2-1）。

### P1-4 融合 SLAM

- **目标**：从 2D slam_toolbox 走向融合（2D 雷达 + 深度/视觉）的 SLAM。
- **现状** `[已核实]`：slam_toolbox mapping mode，`max_laser_range: 15.0`，`resolution: 0.05`。
- **验收**：闭环回环误差可测；`map→odom` 不跳变。
- **风险**：换 SLAM 方案会牵动 `odom_source` 接缝与 Nav2 参数，**应与 P1-1 一起规划**。

### P1-5 全身运动仲裁层（WBC/HQP）

- **目标**：底盘与臂的运动冲突有单一仲裁点 —— `src/rak_wbc` 100 Hz 速度级 HQP
  （Pinocchio + OSQP），成为常规运动的**唯一写入者**（底盘 reference + 臂位置流）。
- **依据**：**ADR-0011**（2026-10-08 已接受）；设计规格见
  `docs/superpowers/specs/2026-10-08-rak-wbc-hqp-design.md`（公式/契约/验收全表）。
- **已做（2026-10-08）**：包骨架 + 数值层 + 桌面回归 15 项全绿（模型雅可比有限差分、
  约束激活、任务冲突、warm-start、降级；含两个历史缺陷回归：`osqp.update` 静默失效、
  posture/EE 权重档位漂移）。节点层与配置已就绪但未上车（`enabled=false` 默认）。
  **影子冒烟已具备并跑通**：`scripts/wbc_shadow_smoke.sh` + `fake_arm_stream` 夹具，
  x86 彩排通过（抓修属性遮蔽 TypeError）；**Jetson 真机影子 2026-10-08 通过**
  （status= solved / 影子输出 / 臂流伺服 / 夹具回显全链路）。
  **时序实测（ADR-0011 #8 闭合）**：Jetson solve p50=2.41ms / p99=2.85ms / max=4.90ms
  （657 样本）——未触发 3ms C++ 迁移判据，Python 方案成立。
  **依赖闭合（#6）**：`ros-jazzy-pinocchio` 4.1.0 (apt) + `osqp` 1.1.3 (pip --user)。
- **推进路线（2026-10-08 重定；物理数字孪生不适用——定义性约束见 `docs/architecture.md` §1）**：
  ① 真底盘 `allow_motion:=false` 影子运行（base reference remap 到影子话题、
  `publish_arm=false`）：验证模型加载 / TF / 100 Hz 时序 / 状态机 / 降级，零运动风险；
  ② wheels-only（P0+P2+P3）上车：测单一写权与 `reference_timeout` 兜底；
  ③ 臂侧留在软件夹具（fake position-stream，mirror `rak_arm_simulation` 模式），
  待 `rak_chassis` 臂接口就绪后增量接 P1；
  ④ 标定冲刺（轮半径/轮距/支撑多边形/质量 CoM/舵机限速）回填占位值（可并行）。
  台架阶梯（轮架空 → 臂 slow 单轴 → Home → 联合低速）不变。
- **验收**：三级矩阵见 spec §9；台架按 `hardware_acceptance.md` 的 Arm Unlock Gates。
- **风险**：臂无关节反馈（命令回读乐观，负载下 EE 误差不可信）；质量/底盘几何未实测前
  倾覆约束只能保守运行；Jetson 侧 `pin`/`osqp` Python 依赖可得性待核实（ADR-0011 #6）。

---

### P1-6 全机坐标系收尾（ADR-0015）

- **已完成**：全机静态骨架入 `rak_chassis.urdf.xacro`（REP-120 `base_footprint`/`base_link` + 四轮实测轮距 27/轴距 25 + 传感器与臂安装帧）；`slam.launch.py` 改由 `robot_state_publisher` 发布、新增 `world→map` 锚点；EKF/恒等里程改发 `odom→base_footprint`。
- **待办**：
  ① **装车标定回填** `TODO(measured)`：**2026-10-14 已回填** `base_height`(0.0275＝轮半径Ø5.5)、
   `lidar_z`(0.1525＝扫描平面离地0.18−base_height)、`mic_z`(0.1325)、`cam_x/z`(0.15/0.0725)、
   `imu_x/z`(0.08/0.0425)＋安装姿态 pitch−90°(芯片顶面朝车后)、`arm_x/z`(−0.15/0.0725)，
   并同步 `wbc_base.urdf.xacro` 镜像；轴距改 0.245、轮距改 0.275（外廓量法：30−轮径5.5 /
   30−轮宽2.5）、nav2 `robot_radius` 按外廓30×30改 0.22、`wheel_radius`×2 改 0.0275、
   `sum_of_robot_...` 改 0.26、WBC `wheel_k`=0.26 / `support_half`=[0.1225,0.1375] 同步
   （wbc_params / node.py / conftest 三处）。**剩余**：外壳尺寸（visual 盒仍占位）。
   ⚠️ 同日实测 IMU 无应答：四路 spidev `WHO_AM_I=0x7f`（非 0x6b），查接线/pinmux。
  ② **REP-120 迁移上车复测**：`odom→base_footprint` 后 EKF 收敛、rf2o 里程、Nav2 跟踪与 `collision_monitor` 需实机验证；`ros2 run tf2_tools view_frames` 确认每帧唯一发布者。
  ③ **`world→map` 全局锚点**：现为静态恒等占位，待全局定位/多地图注册模块（无限开放世界）。
  ④ **相机外参** `base_link→camera_link` 按 ADR-0014 实测（不得臆造）。
- **运行约束**：nav 栈与 `chassis_bringup` 同跑时，`slam.launch.py` 需 `publish_robot_tf:=false`，避免两个 `base_footprint→base_link` 发布者。

## P2 — 感知栈与算力

### P2-1 算力分配裁定（边缘 vs 远程）

- **目标**：明确哪些跑在 Orin Nano 4GB，哪些放远程 x86 工作站。
- **依据** `[已核实]`：`docs/architecture.md` §5 —— "约 3.4 GB 可用总内存的设备
  不能默认同时常驻所有模型与 GUI"。
- **已定**（ADR-0004 D3）：**留在设备**——传感器驱动、安全实时环、Nav2 局部代价地图+控制器、
  主里程计、DeepStream 单路 detect+track、STVL。
  **外移工作站**——MoveIt2 规划、SLAM 回环/位姿图、DeepStream 二级分类器、任何 VLM、RViz2/MoveIt GUI。
- **验收**：给出**按场景的能力矩阵**（导航 / 抓取 / 演示各需要哪些节点常驻）+ 实测 RSS/GPU/CMA。
- **风险**：**这是 P2/P3 一切的前置** —— 不裁定就等于默认"全都要"，必然 OOM 或掉帧。

> **⚠️ 内存策略待复测（挑战 2026-10-05 的主机配置）。**
> 研究结论对 4 GB 板上的 **zram** 持反对意见："zram 消耗的正是它要扩展的那块内存池"，
> 并建议"swap 只作 OOM 保险，持续 swap 即说明配置错了；4 GB 板上 swap 会导致数秒卡顿"。
> 但 2026-10-05 已在主机上启用 **zram（zstd 50% ≈1.7 G）+ 4 G swapfile**。
> 两方说法都需要**实测裁决**：跑一次真实负载（Nav2 + SLAM + 相机），
> 观测 `zram` 压缩率/CPU 占用、`si/so`、以及端到端延迟 p99。
> 判据：若持续 swap 或 p99 明显劣化 → 关 zram、缩 swapfile；否则保留。
> **不要凭文档直接改回**，这是有争议的经验结论，不是定论。

### P2-2 DeepStream 9.1 / Holoscan 4.6 可行性

- **目标**：判断这两个栈在 JetPack 7 / Orin Nano 4GB 上是否可用，以及值不值得。
- **已定**（ADR-0004）：**DeepStream 9.1 = GO**（官方配对正是 JetPack 7.2 / L4T 39.2，
  与本机完全吻合；须用 IOU tracker、headless、C++、不做 RTSP 回推）。
  **Holoscan 4.6 = NO** —— Orin 系列要求 **JetPack 6.2.1**，JP 7 上它只支持 AGX Thor，
  **与 DeepStream 9.1 在本板互斥**。
- **验收**：一份结论 + 最小可运行样例（或明确的"不可行"证据）。
- **风险**：4GB 机型上 DeepStream 很可能**不能与 Nav2/SLAM 同时常驻**。

### P2-3 深度点云管线

- **目标**：Astra 深度 → 点云 → 可消费的 `sensor_msgs/PointCloud2`。
- **现状** `[已核实]`：`rak_astra_driver` 存在（OrbbecSDK 1.10.18），但
  **`pcl_ros` 未安装**（已核实缺失）。
- **前置**：相机标定、深度-彩色对齐（`docs/architecture.md` §7 列为未解决）。
- **验收**：点云在 RViz 中与彩色对齐；地面平面拟合误差可测。

### P2-4 视觉 + 点云融合

- **目标**：2D 检测/分割结果与 3D 点云关联，产出带 3D 位姿的目标。
- **现状** `[已核实]`：`docs/architecture.md` §7 —— "深度/检测/跟踪融合**尚未形成实际
  `TrackedObjects` 生产链**"。
- **验收**：`/perception/tracked_objects` 有真实数据（现在是空的），字段与
  `rak_interfaces` 契约一致。

### P2-5 超远物体建模

- **目标**：远距离目标（超出深度相机有效范围）的建模，供后续接近/抓取。
- **候选**：雷达点云 + 视觉三角化；先接近再建模。
- **风险**：**依赖机械臂工作空间**（当前臂的几何/限位还是测试占位值），
  目标定义可能随臂实机数据变化。

### P2-6 语音模块

- **目标**：对话入口（ASR/TTS），归入 `docs/architecture.md` 的 **E 阶段**。
- **前置**：**先以文字事件验证任务控制**，再接语音 ——
  避免"语音识别问题"与"任务控制问题"混在一起（`docs/architecture.md` §1 已定此原则）。
- **现状**：文字 CLI（`task_cli`）已有；ASR/TTS 未接入。
- **风险**：普通打断需要**快速本地识别路径**，不能等云端完整推理；
  但语音**不是**硬件急停的替代品（`docs/architecture.md` §4）。

### P2-7 Harness 与「呼吸感」表达层

- **依据**：**ADR-0010**（已接受，设计先行，未实现）。
- **目标**：把"云端 LLM + Harness 理解任务"从愿景落成软件；Jev（S1）经
  `/brain/decision` 影响任务，但**只传递、不赋权**。
- **要做**（全部软件、不依赖硬件门）：
  1. 建标准 ROS 2 包 `src/rak_harness` + 节点 `harness_node`（console entry point），
     由 `rak_bringup` 与任务栈一并启动。
  2. 在 `src/rak_interfaces` 新增 `HarnessRequest` / `HarnessProposal` / `HarnessEvent`
     三份消息（`schema_version=1`），话题 `/harness/request`、`/harness/proposal`、`/harness/event`。
  3. 执行器 `slow_planner` 参数扩展 `local | rai | harness`；`harness` 走
     "发布请求 → `PLANNING` 内限时等待匹配提案"，接受前校验
     `task_id`/`revision`/`expires_at`/白名单。
  4. 新 `anthropic` 客户端（`rak_harness/cloud_client.py`）：`base_url=https://api.deepseek.com/anthropic`、
     key 取 `ANTHROPIC_API_KEY`、tool-use 结构化输出、单工作线程 + 容量 1 队列。
  5. 表达层 `ExpressionIntent`/`ExpressionGate`/`NoOpExpressionSink`（本轮恒 no-op）。
- **前置 / 待实测**：`anthropic` SDK 在系统 Python 的安装与版本；DeepSeek Anthropic 端点的
  可用模型 id（`deepseek-chat` 为占位）；tool-use 行为。
- **验收**：① `/harness/proposal` 的陈旧（revision 变更）、过期（`expires_at`）、
  预算耗尽、Harness 重启四类竞态测试；② 未装 `anthropic` 时降级为只发 `KIND_NO_OP`；
  ③ harness 路径端到端延迟 p50/p95/p99；④ 表达层门恒关（`ExpressionGate` 恒 false）。
- **风险**：3.4 GB 板上新增进程与依赖；话题式 request/proposal 活动部件多，
  正确性全押在 `task_id`/`revision`/`expires_at` 三件套上。

---

## P3 — 抓取

> **前置硬门槛**：`hardware_acceptance.md` §Arm Unlock Gates 未通过前，
> 臂只能用软件夹具，**不得配置真实 `ros2_control` 硬件插件**。
> 以下 P3 全部是**算法侧**准备，可先于臂实机推进。

### P3-1 点云几何抓取（先做）

- **目标**：不依赖神经网络，用点云的几何/**反极点**分析产出候选抓取位姿。
- **候选**：自写反极点检测（法线 → 反极点对 → 工作空间/IK 过滤）/ GPD（BSD，CPU）。
- **验收**：在已知物体上产出可执行的抓取位姿，成功率可测。
- **理由**：**最轻、最快见效**的一档，先建立"点云 → 抓取位姿 → MoveIt 执行"的完整链路。
- 说明：末端是**舵机夹爪（平行夹爪）**，反极点范式成立（ADR-0008 曾误记为吸嘴，已更正）。
  `moveit_grasps` 仍**不可用**（Jazzy 上 blocked），用 `moveit_task_constructor` 做 pick/place 时序。

### P3-2 抓取网络（GraspNet / AnyGrasp）

- **目标**：学习型抓取位姿生成。
- **已定**（ADR-0005）：**优先 GraspNet-1Billion**（非商业许可、<1 GB 显存、~1–3 Hz）；
  **AnyGrasp 暂缓**（仅研究用途、按机器指纹锁定、MinkowskiEngine 在 CUDA 13 上移植成本高、无 TRT 路径）。
- **验收**：与 P3-1 在同一测试集上对比成功率与延迟。
- **风险**：**4 GB 显存/内存是硬约束**；可能需要远程推理。

### P3-3 大语言模型 / VLM 抓取

- **目标**：自然语言指定目标（"抓红色的杯子"）→ 视觉定位 → 抓取位姿。
- **候选**：VLM 选目标 + P3-1/P3-2 出抓取位姿。
- **约束** `[已核实]`：`src/rak_agent/rak_agent/types.py` 是**冻结契约**，`state_text` 逐字存进
  episode JSONL。新任务协议必须**另行版本化**，不得改写冻结语义。
- **风险**：S2 延迟 12–30 s，**不能进实时控制环**（`docs/architecture.md` §4）。

### P3-4 臂实机适配（标准 6 轴 + 舵机夹爪）

- **前置**：拿到真实几何/限位/映射（`hardware_acceptance.md` §Arm Unlock Gates 七项）。
- **形态**（ADR-0008）：**标准 6 轴** + **舵机夹爪**（第 7 个 PWM 舵机）+ 末端摄像头。
  现有五轴 URDF 与六轴夹具**都要重建**。
- **进度 [2026-10-08]**：固件 R42 曾打通 USB v2 `ARM_COMMAND`（7 轴 µrad）→ 舵机 PWM
  落地（轴序/限位/零位见下位机 `app_car.h`）。USB v2 链路与直连示教 TUI 已按 ADR-0016
  （2026-10-14）删除；臂命令待改走 CAN（7 轴 `ARM_COMMAND` 接口未实现，ADR-0012/0016
  待闭合项），示教工具按 ADR-0016 决策 2（只经 ROS 接口）重建。**剩余**：`rak_chassis`
  ros2_control SystemInterface 插件 + `joint_trajectory_controller` 接入。
- **验收**：`ros2_control` 硬件接口 + `JointTrajectoryController`，插值/刷新率按固件定。
- **风险**：**不要把 25 kHz 电机环搬到 ROS**（`docs/architecture.md` §4）。
- ⚠️ **舵机角度 ↔ 钳口开度是非线性的**（连杆机构）—— 需要映射曲线，建议放上位机（改曲线不烧固件）。
  夹爪**无夹持力反馈**（位置舵机），抓取成功与否要靠视觉确认。

### P3-5 末端摄像头与 chArUco 手眼标定

- **目标**：eye-in-hand 近距感知，供抓取前的局部精定位与抓取确认。
- **要做**：相机选型 → **内参标定** → **chArUco 手眼标定**（eye-in-hand，`AX=XB`）→ 与车体深度相机坐标统一。
- **注意**：⚠️ **本仓目前没有末端相机的任何痕迹**（已核实），需**从零新增** URDF link + 驱动 + 标定。
- **风险**：多一路视频流会挤占 4 GB 预算（ADR-0004，已无余量）；标定误差直接进抓取精度。

### P3-6 URDF 重建与关节零位标定

- **目标**：把 `six_axis_sim.urdf.xacro` 的**虚构数值**换成实测参数，成为真实模型。
- **依据**（ADR-0009）：**不需要精确 CAD** —— 规划只要运动学（joint origin/axis/limits），
  碰撞体用**图元保守近似**，惯量仅仿真需要，视觉体与规划无关。
- **进度 [2026-10-08] 基本完成**：`src/rak_description/urdf/rak_arm.urdf.xacro`
  （真实参数版，与 fixture 隔离）+ `rak_moveit_config/config/rak_arm.srdf`。
  实测标定：六轴位置型，J1/J3/J4/J5/J6 0~270°、J2 0~255°（硬件约束）；连杆
  8/10/11/6/9 cm；零位（q=0）= 指令 J1/J4/J6=0°、J2=135°、J3=180°、J5=0°；
  J3 俯仰方向与 J2/J5 反向；俯仰轴在中位同轴共线、各 ±90°。常数单点定义在
  `properties_arm.xacro`，`test/validate_rak_arm.py` 断言（colcon test 已挂）。
  设计与事实分级见 `docs/superpowers/specs/2026-10-07-rak-arm-moveit2-design.md`。
- **剩余**：① 零位精标（当前为用户目测指定）；② MoveIt 规划配置
  （joint_limits/kinematics/ompl/mock controllers + launch，阶段 A M3/M4）；
  ③ 夹爪/末端摄像头 link（待支架确定）。
- **验收**：MoveIt 能对真实目标位姿规划出可行轨迹；`tf2_tools view_frames` 链路正确。
- ⚠️ **关节零位标定是独立于手眼标定的另一件事**，两者都要做，否则整个 TF 树是偏的。

---

## 与阶段 A–F 的映射

| 本文档 | `docs/architecture.md` 阶段 | 关系 |
|---|---|---|
| P0-1…P0-4 | **D：硬件闭环** | 同一件事的不同侧面 |
| P0-5, P1-* | **C：Nav2 接入**（扩展） | 融合与避障是 C 阶段质量提升 |
| P2-* | 未编号（新增） | 感知栈此前只有驱动，无融合链 |
| P2-7 | **E：语音与智能体** | Harness/仲裁/表达层，见 ADR-0010；纯软件，不依赖硬件门 |
| P3-* | **F：机械臂** | 算法侧先准备，实机受 F 阶段门槛约束 |

---

## 明确不做（避免重复讨论）

- **不装实时内核**：闭环在 MCU，Jetson 用出厂 `PREEMPT` 足够（见 `docs/CONSTRUCTION.md` §9）。
- ~~不在 Jetson 上做麦轮逆运动学~~ → **已反转，见 ADR-0007**：麦轮 IK **移到上位机**
  （`mecanum_drive_controller`），F407 只做每轮 PID + 舵机 PWM。
- ~~不用 `mecanum_drive_controller`~~ → **已反转，见 ADR-0007**：现在正是用它。
- **不让 S2/云端 LLM 进实时控制环**：延迟 12–30 s，只能 fire-and-forget。
- **不让 Jev（S1）决策直接驱动任务执行器**：见 ADR-0010 —— 只经 Harness 传递、不赋权；
  执行器**不**订阅 `/brain/decision`，Harness 只输出结构化提案，永不发速度/关节命令。
- **不共用 LLM key 变量**：RAI 慢路径用 `OPENAI_API_KEY`，Harness 用 `ANTHROPIC_API_KEY`。
- **不改 `src/rak_agent/rak_agent/types.py` 的冻结语义**：要新协议就版本化。
- **不在本仓定义 CAN 帧字段**：CAN v3 的源在下位机仓（ADR-0003）。

---

## 附：本机已完成的主机侧变更（2026-10-05）

非仓库代码，换板/重装需重做，记录在 [`docs/CONSTRUCTION.md`](docs/CONSTRUCTION.md) §2.2：

- SSH 加固（禁密码登录，仅公钥）；移除 snap（含 firefox/thunderbird）+ 44 个无关包；
- zram（zstd 50%）+ 4 GB swapfile —— **改变了 `CLAUDE.md` 记录的内存前提**；
- `nvpmodel -m 0` + `jetson-clocks` 开机自启；
- 40-pin UART udev 别名 `/dev/robot_uart` → `ttyTHS1`
  （⚠️ 与底盘链路无关——底盘走 CAN v4（ADR-0016），该 40-pin 别名应清理或明确其归属）。
