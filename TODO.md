# TODO — 感知 / 融合 / 抓取 路线图

- **日期**: 2026-10-05
- **状态**: 规划中（未开始实施）
- **配套**: 决策理由见 [`docs/adr/`](docs/adr/README.md) · 装配顺序见 [`docs/CONSTRUCTION.md`](docs/CONSTRUCTION.md) ·
  系统结构见 [`docs/architecture.md`](docs/architecture.md) · 上车门槛见 [`docs/hardware_acceptance.md`](docs/hardware_acceptance.md)

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
| **P3** | 抓取（几何 → 网络 → VLM） | 机械臂作业 | P2 + 臂实机解锁 |

---

## P0 — 阻塞项

### P0-1 迁移到统一 `ros2_control` 架构（当前最大任务）

- **目标**：全机改走 `ros2_control` —— 底盘 `mecanum_drive_controller`、
  机械臂 `joint_trajectory_controller`、一个 `hardware_interface::SystemInterface` 插件走 USB。
- **依据**：**ADR-0007（取代 ADR-0001）**。硬件前提已变：只剩一块 F407 同时管 4 轮 + 6 舵机，
  CAN 失效，Jetson 只走 USB。
- **要做**：① 写 `rak_hardware` C++ 插件（4 轮 velocity + 6 关节 position）；
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
- **验收**：`node.py` 发 200 ms lease，实车行驶 5 min 无 lease 超时锁存；拔线后 MCU 在
  lease 内独立停止。
- **风险**：200 ms 意味着**断联后底盘最多再走 200 ms** —— 高速下的滑行距离必须核算。
- **关联**：ADR-0003 §A1

### P0-3 ~~裁定"Jetson 是否在 CAN 总线上"~~ ✅ 已裁定

- **结论（2026-10-05 用户确认）**：**Jetson 只通过 USB 与下位机通信，不接 CAN。**
  臂板烧毁后 CAN 对端不存在，CAN 层整体失效。
- **遗留动作**：把 `src/rak_can_driver` 与 `src/rak_serial_link` 明确标记为
  **废弃 / 保留待用**（当前是"存在但语义悬空"）。TJA1050 电平问题（ADR-0003 §A7）随之作废。
- **关联**：ADR-0003 §A3（已更新）、ADR-0007

### P0-4 底盘 UART 实车验收

- **目标**：从"launch 能解析"走到"真实 F407 上跑通"。
- **前置**：下位机固件确认（build_tag / 协议版本 / feature bits）。
- **验收**：严格按 [`hardware_acceptance.md`](hardware_acceptance.md) §Staged Base Test 六步：
  PTY 回归 → 只读握手 → 拔线/超时/E-stop 撤销 → 轮子离地单轴低速 → 软件暂停与物理急停分开 →
  围栏地面测试。
- **风险**：上位机文档仍写"仓库没有匹配的 F407 固件"，**该声明已过期**（ADR-0003 §A5），
  会误导验收准备。先更正文档。
- **关联**：ADR-0003 §A5、`docs/CONSTRUCTION.md` §6

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

- **目标**：轮式 + IMU + 雷达（rf2o）+ 视觉 四路融合，输出可信的 `odom→base_link`。
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
  否则**两个节点抢发 `odom→base_link`**。
- **验收**：`ros2 launch rak_navigation slam.launch.py odom_source:=ekf` 后
  `ros2 run tf2_tools view_frames` 只有一个 `odom→base_link` 发布者。

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

---

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
- **约束** `[已核实]`：`brain/rak_ai/types.py` 是**冻结契约**，`state_text` 逐字存进
  episode JSONL。新任务协议必须**另行版本化**，不得改写冻结语义。
- **风险**：S2 延迟 12–30 s，**不能进实时控制环**（`docs/architecture.md` §4）。

### P3-4 臂实机适配（标准 6 轴 + 舵机夹爪）

- **前置**：拿到真实几何/限位/映射（`hardware_acceptance.md` §Arm Unlock Gates 七项）。
- **形态**（ADR-0008）：**标准 6 轴** + **舵机夹爪**（第 7 个 PWM 舵机）+ 末端摄像头。
  现有五轴 URDF 与六轴夹具**都要重建**。
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
- **阻塞项**：需要**关节 origin / axis / limits** 三项实测值（其余可后补）。
- **验收**：MoveIt 能对真实目标位姿规划出可行轨迹；`tf2_tools view_frames` 链路正确。
- ⚠️ **关节零位标定是独立于手眼标定的另一件事**，两者都要做，否则整个 TF 树是偏的。

---

## 与阶段 A–F 的映射

| 本文档 | `docs/architecture.md` 阶段 | 关系 |
|---|---|---|
| P0-1…P0-4 | **D：硬件闭环** | 同一件事的不同侧面 |
| P0-5, P1-* | **C：Nav2 接入**（扩展） | 融合与避障是 C 阶段质量提升 |
| P2-* | 未编号（新增） | 感知栈此前只有驱动，无融合链 |
| P3-* | **F：机械臂** | 算法侧先准备，实机受 F 阶段门槛约束 |

---

## 明确不做（避免重复讨论）

- **不装实时内核**：闭环在 MCU，Jetson 用出厂 `PREEMPT` 足够（见 `docs/CONSTRUCTION.md` §9）。
- ~~不在 Jetson 上做麦轮逆运动学~~ → **已反转，见 ADR-0007**：麦轮 IK **移到上位机**
  （`mecanum_drive_controller`），F407 只做每轮 PID + 舵机 PWM。
- ~~不用 `mecanum_drive_controller`~~ → **已反转，见 ADR-0007**：现在正是用它。
- **不让 S2/云端 LLM 进实时控制环**：延迟 12–30 s，只能 fire-and-forget。
- **不改 `brain/rak_ai/types.py` 的冻结语义**：要新协议就版本化。
- **不在本仓定义 CAN 帧字段**：CAN v3 的源在下位机仓（ADR-0003）。

---

## 附：本机已完成的主机侧变更（2026-10-05）

非仓库代码，换板/重装需重做，记录在 [`docs/CONSTRUCTION.md`](docs/CONSTRUCTION.md) §2.2：

- SSH 加固（禁密码登录，仅公钥）；移除 snap（含 firefox/thunderbird）+ 44 个无关包；
- zram（zstd 50%）+ 4 GB swapfile —— **改变了 `CLAUDE.md` 记录的内存前提**；
- `nvpmodel -m 0` + `jetson-clocks` 开机自启；
- 40-pin UART udev 别名 `/dev/robot_uart` → `ttyTHS1`
  （⚠️ 与项目约定的 USB-UART / `/dev/wheel_motor` 不一致。按 ADR-0007，底盘走 **USB**，
  这个 40-pin 别名与底盘链路无关，应清理或明确其归属）。
