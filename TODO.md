# TODO — 工作队列

> 状态：现行 · 最后更新：2026-11-16

本文件是本仓**唯一**的工作队列。规则：

1. 只写"做什么 + 怎么判过 + 依据在哪"；细节、命令、验收脚本在依据文档里，不复制。
2. 做完打 `[x]` 并在括号里写实测结论（命令 + 关键输出），**不删条目**；
   一轮结束把已闭合条目移进 `docs/status/project_status.md` 对应日期的增量段。
3. 没跑过的验证一律写"未验证"，不写"应该没问题"。
4. 优先级看阻断关系：P0 挡住真机，P1 挡住定量结论，P2 是质量升级，P3 是远期。
5. 进度叙述只在这里和 `docs/status/project_status.md`，其他文档只链接。

## P0 — 挡住真机

2026-11-16 真机端到端联调（已完成，证据见 `docs/status/project_status.md` 当日增量段）：

- [x] **CANopen 单主站**：总线上只有一个 `device_container`；被动监听只有节点1 心跳
  `701=05`、无第二主站发帧；SYNC `080` 1488 帧/3 s ≈ 496 Hz，4 路 TPDO 各 ~100 Hz。
- [x] **R76 固件在板**：SDO `0x2F00`=3553106（0x363352 → R76）、`0x2F01`=66（`'B'` 掉电复位）。
- [x] **`/cmd_vel` → wbc → `/mecanum_drive_controller/reference`**：本机发 `vx=0.05, wz=0.2`，
  orin 侧 reference `linear.x=0.049995`、`frame_id=base_link`，wbc `enabled/nav`。
- [x] **Nav2 规划 + MPPI 下发**：`NavfnPlanner` + `nav2_mppi_controller`；
  `ComputePathToPose` `error_code=0`，`NavigateToPose` 后 `/cmd_vel` 抓到 232 条（`x≈0.0051`）。
- [x] **C30D 断电重上电后控制环恢复**：`0x2034`（imu_age）由饱和 255 → 10，
  `0x2020` 位置从冻结值恢复更新；心跳 OPERATIONAL、无 EMCY。

- [ ] **四轮不响应速度脉冲**（2026-11-16，地面实动）
  为什么卡着：`allow_motion=true`、`mecanum_drive_controller=active`、
  `/mecanum_drive_controller/reference=0.03` 已确认，但 1.5 s 脉冲内四轮
  `/joint_states.velocity` 只有 ±0.01 rad/s（约一个编码器 LSB），期望
  `0.03/0.01375≈2.18 rad/s` 且四轮同号；`gyro.z` 无有效偏航（`wz_max_abs=0.049`）。
  决定了会影响：能否进入真机导航；也决定是改标定/量纲还是查驱动使能。
  下一步：脉冲期间同步抓 `/chassis/tpdo` 的 `0x2000..0x2003`，与 reference 对比，
  区分「量纲/比例错误」与「驱动器未真正使能」。
  依据：`docs/reference/canopen_bitfields.md`、`TODO.md` 当日联调记录。

- [ ] **四轮 TF NaN**
  为什么卡着：底盘硬件只有 velocity 接口，`/joint_states.position` 四轮恒为 NaN，
  `robot_state_publisher` 按 position 算 TF 输出 NaN，wbc 报 `TF_NAN_INPUT`，
  轮 link 在 TF 树里不可用。
  决定了会影响：RViz/代价层的轮式可视化、任何依赖轮 TF 的检查，以及轮速的交叉核对。
  下一步：底盘侧积出轮角位置再发布，或改 URDF 让轮关节不依赖 position。
  判过：`tf2_echo base_link <wheel>_link` 输出有限值，且随轮子转动连续变化。

- [x] **R76 烧录 + 冷启动核验**（底盘板 `~/vehical_rak_motion`，2026-11-16 完成）
  构建双 preset 零警告、`tools/canopen_gate.py` 绿、tag `R76` 均已完成；
  烧录后冷启动实测：`0x2F00`（build_tag）= 3553106 = 0x363352 → **R76**，
  `0x2F01`（reset reason）= 66（`'B'`，掉电复位）。
  依据：`../vehical_rak_motion/docs/CANOPEN-V5.md`、`docs/status/project_status.md`。
  已就绪产物：Debug `abeea499f6a14924` / Release `58ee9eb151fe08a7`（SHA-256 前 16）。

- [x] **ros2_canopen 装机 + vcan 跑通**（2026-11-14）
  apt 0.3.4（core/master-driver/proxy-driver/ros2-control/fake-slaves）装好；
  `python3 scripts/verify_canopen_vcan.py` exit 0：NMT reset → boot-up → SDO 读 → NMT start，
  `/master/sdo_read` 成功。`master_config` 必须是 `dcfgen` 生成的主站 DCF（已入档）。
  依据：`docs/status/canopen_vcan_2026-11-14.md`、ADR-0024 修订（2026-11-14）。
- [x] **SYNC / TPDO / `~/rpdo` 全链闭合**（2026-11-15 真机）
  根因不是"主站不发 SYNC"（它发，`080#` 稳定 500 Hz），而是 `dcfgen` 缺官方 `-r`
  （remote PDO）：主站 OD 没有 0x5800/0x5A00，lely 就不把从站 TPDO 当 RPDO 写入。
  已修：`gen_canopen_master_dcf.py` + `test_dcf_sync.py` + `verify_canopen_vcan.py` 全部加 `-r`。
  另修两个下位仓 `gen_eds.py` 的 PDO COB-ID（必须 `$NODEID+0x…`）。
  实测：`/arm/rpdo` 505 Hz，`/joint_states` 真实读数。见 `docs/status/canopen_vcan_2026-11-14.md`。
- [x] **底盘板（node 1）接入**：2026-11-16 已恢复。心跳 `701=05`（OPERATIONAL）、
  SDO 读写正常、4 路 TPDO 各 ~100 Hz；`0x2F00` 读回 R76。
  断电重上电后控制环恢复（`0x2034` imu_age 10、`0x2020` 位置恢复更新）。
  注意：速度脉冲仍不跟踪，单列为 P0「四轮不响应速度脉冲」。
- [ ] **`j6_wrist_roll` 零位核对**：实测回读 4.712 rad，而 `arm_controller` 目标 0，
  controller_manager 持续报 out of limits。确认 j6 真实零位后回填契约/URDF 限位。
- [ ] **真机 `can0` 首帧核对**（装机后第一件事）
  每个 PDO 的 DLC 与契约 `byte_size` 一致、没有未知 COB-ID、物理量在合理范围、
  字节序证据回填；`0x2F00` 读回 R76/A16；SYNC 2 ms 抖动与心跳消费实测。
  在此之前 `rak_chassis` 薄桥只有 gtest 与 URDF 契约证明，**端到端未验**。
  依据：`docs/adr/0024-canopen-master-on-jetson.md` §5 与 2026-11-14 修订、
  `docs/adr/0022-jetson-canopen-contract-layer.md`、`docs/status/next_2026-10-30.md` §5。

- [ ] **机械臂真机 J2–J6 运动 + 限位 + 运动中急停**（J1 已验，其余未验）
  依据：`docs/reference/hardware_acceptance.md` 的臂侧门禁、
  `docs/reference/arm_moveit_contract.md` §1。

## P1a — 视觉寻路 v1.0（`docs/design/vision_nav_v1.md`）

已完成（2026-11-15，均有实测）：
- [x] **感知换 TensorRT**：`rak_road_perception` 去 torch，`roadseg_trt.py` 用 ctypes
  直连 TRT+libcudart；TwinLiteNet+ Small 384×672 FP16 = 15.9 ms GPU / 25.4 ms 单帧。
  工具：`scripts/export_twinlite_onnx.py`、`build_roadseg_engine.py`。
- [x] **A/B 对照**：TwinLiteNet+ 40.3 ms/24.8 FPS、边界锐度 soft/bnd 2.05；
  LFD-RoadSeg v6 190.8 ms（p50 66.5）/5.2 FPS、12.51。工具 `scripts/compare_roadseg.py`。
- [x] **前视相机接入**：`/dev/video0` 单目 UVC；`v4l2_camera` 不支持 MJPG，改官方
  `usb_cam` MJPG 1280×720@30（实测 29.4 Hz / 单核 50.5% / RSS 90 MB）。
  `launch/camera.launch.py`。
- [x] **单目标定**：ChArUco 5×7（板型由 0.70 px vs 97 px 残差测出）、24 视角、
  in-sample 0.401 px / hold-out 0.381 px、fx=844.40 fy=851.46 cx=614.20 cy=401.18。
  `scripts/calibrate_camera_charuco.py`，产物 `~/.ros/camera_info/front_camera.yaml`。
- [x] **RTAB-Map 纯定位 + AprilTag 骨架**：apt 装 rtabmap 0.23.13 + apriltag_ros 3.4.0；
  `launch/localization.launch.py` + 两个 config。干跑确认
  `Localization mode (Mem/IncrementalMemory=false)`；apriltag 合成图 44/45 帧检出。
- [x] **Nav2 收口**：`sensors:=camera|map_yaml:=` 旁路；MPPI 加 TwirlingCritic、
  vy_std 0.08 / wz_std 0.12。
- [x] **DeepStream/Holoscan 实测**：单相机 29.9 FPS；batch=6 只赚 15%（访存受限）；
  `nvv4l2decoder` 解 MJPG 失败；Holoscan 无 Python 绑定。
- [x] **内存预算**：roadseg TRT 节点 625 MB（实测）；含 qwentts 全栈 4.1–5.4 GB > 3.4 GB。
- [x] **Gemini 335L 接入（2026-11-16）**：按官方 `ros-yazzy-orbbec-camera` 2.9.3 适配，
  深度 10.13 Hz / 彩色 4.77 Hz / 相机 IMU 187 Hz，直连 USB3；Astra 路径保留。
- [x] **全模型下载齐备**（2026-11-15）：`acoustic/models/` 下 kokoro-int8（207 MB）
  + streaming-zipformer-zh-14M（78 MB）；`data/models/laya-typed-decisions/` 下
  safetensors（804 MB）→ BF16 ONNX（807 MB 含 data）→ **TensorRT 引擎 807 MB**
  （构建 65 s，`trtexec PASSED`）；`test_tensorrt_decider.py` 5 项全过。
- [x] **全模型高负载实测**（2026-11-15）：八组件同跑（Laya TRT + roadseg TRT +
  TTS + ASR + Nav2 + rtabmap + apriltag + camera），组件合计 **2990 MB**，
  整机峰值 **3455/3520 MB**、最低可用 **64 MB**、swap 峰值 **2049 MB**（满）。
  Laya 单进程 **1616 MB** 是最大单体。**结论：Laya 与全栈同时常驻不可行**，
  必须按需加载或二选一。Laya TRT 实测 **67.3 ms**，ASR 48–70 ms/句。
- [x] **语音交互延迟预算实测**（2026-11-15）：唤醒 <50 ms + ASR 48–70 ms/s +
  端点检测 0.5–1.2 s + Laya 67 ms + 即时层 <100 ms ≈ **1.7 s 首响**（可行）；
  内容层 kokoro 2.2–2.9 s 首响（**不可行**，ADR-0018 三层结构是唯一达标路径）。
  依据：§5A.12。
- [ ] **ASR 部分结果驱动动作**（把 1.2 s 端点等待压到 ~0.3 s）：`asr_node` 边解码
  边发中间意图，高置信度直接执行、低置信度只播"嗯？"。这是"无感"的关键一步。
- [ ] **Laya 按需加载**：全栈下 64 MB 可用内存压线，需改为收到决策请求再加载引擎
  （或与 roadseg 共享 CUDA 上下文）。
- [x] **全栈高负载实测**（2026-11-15，`scripts/loadtest_full_stack.py`）：usb_cam +
  roadseg(TRT) + apriltag + rtabmap 纯定位 + Nav2 4 服务器 + kokoro TTS 同跑，
  Nav2 达 `Managed nodes are active`；组件合计 **2051 MB**，整机峰值 **2869/3520 MB**，
  最低可用 650 MB，swap 峰值 756 MB（zram 压缩后约 200 MB），TTS RTF 2.31–2.41 稳定。
  **顺手修掉一个真 bug**：`nav2.launch.py` 的 `sensors:=camera` 分支不启
  `robot_state_publisher`，导致 `base_link→odom` 缺失、Nav2 生命周期 `Aborting
  bringup`（整栈看似起来实则全 inactive）。已修。依据：§5A.10。

待做：
- [ ] **贴 AprilTag 才能重定位**：实测画面里只有 ChArUco 标定板，无 36h11 tag，
  `/detections` 为空数组。需打印并贴在已知坐标（§3.1c 布设表）。
- [ ] **rtabmap 首次真机重定位**：判据 = 与 AprilTag PnP 位姿差 <5 cm（静态 30 s）。
- [ ] **离线建图一次并落盘**（N10_P 雷达 + `slam.launch.py`），存 `data/maps/`，
  作为 `sensors:=camera map_yaml:=` 的输入。
- [ ] **MPPI 直道实测**：直道 10 m 往返横向偏差 <5 cm。
- [ ] **TTS 换 kokoro**（`backend:=sherpa`）：实测 RSS 408 MB（vs qwentts 1.2–1.5 GB），
  RTF 2.2；TRT/CUDA EP 对 TTS 无收益（见设计 §5A.9）。模型下载
  `kokoro-int8-multi-lang-v1_1`（147 MB），放 `acoustic/models/`（gitignored）。
- [ ] **六相机前解决 MJPG GPU 解码**：否则 6×50% CPU ≈ 3 核，第一瓶颈。
- [ ] **深度面板彩色化**：RViz 深度显示是否改成彩色伪彩面板未定，等使用方拍板
  （当前 `sensors_rviz.rviz` 保持原样，不与深度显示那条线并行改）。
- [ ] **标定质量复核（可选）**：板铺满整帧四角再标，看 k3 是否变有意义。

## P1a-2 — 4D 融合视觉导航 v2.0（`docs/design/vision_nav_v2.md` / ADR-0027）

已完成（2026-11-15，门禁全绿；**真机未验证**）：
- [x] **EKF 四路融合**：`config/ekf_4d.yaml`（轮式差分 + rf2o 绝对 + VO 差分 + IMU vyaw），
  `slam.launch.py ekf_params:=` 透传。gtest 等价门禁：`test_4d_nav_config.py` 8 项。
- [x] **视觉里程计**：`launch/vio.launch.py`（`rtabmap_odom rgbd|stereo` → `/odometry/vio`，
  `publish_tf: false`）；松耦合 VIO = VO + EKF 融陀螺。
- [x] **4D 代价层**：`config/nav2_4d.yaml`，`rtabmap_costmap_plugins::VoxelLayer`
  （局部 `decay=5 s`，全局持久），三源 `/scan` + `/camera/depth/points` + `/stereo/points`。
  STVL 2.5.5 实测不标记（同数据同场景 0 格），已记入 ADR-0027「被否决/待闭合」。
- [x] **全向寻路**：`SmacPlannerLattice` + 上游 omni 原语（5 cm/0.5 m），
  configure 实测 `Using motion model: State Lattice`；MPPI Omni 沿用。
- [x] **单入口 + 门禁**：`ros2 launch rak_bringup 4d_nav.launch.py`；
  `scripts/verify_4d_fusion.py` 6 道门全绿（G3 实测标 1549 格、G5 rtabmap VO 出
  `/odometry/vio`、G6 `ComputePathToPose` SUCCEEDED 202 点），合成传感器 fixture
  `scripts/fake_4d_sensors.py` + `scripts/fake_vio_sensors.py`。

待做：
- [ ] **真机首帧**：深度相机 + 雷达同时进体素层（`topic hz` + costmap 标记），判据见 ADR-0027 待闭合 1。
- [ ] **EKF 协方差标定**：Allan + 直行 2 m，回填 `ekf_4d.yaml`（同 P1b）。
- [ ] **双目标定 → `/stereo/points`**：`stereo_image_proc` 已装未接线。
- [ ] **全栈内存实测**：含体素层与 VIO 的 RSS/最低可用内存；超 3.4 GB 就砍全局体素层。
- [ ] **STVL 重测**：查明不标记原因，通过则换回（OpenVDB 稀疏网格省内存）。
- [ ] **cuVSLAM 对照**：Isaac ROS 4.6（官方支持 JP7.2 + Jazzy，走 NVIDIA apt + `isaac-ros-cli`）
  与 rtabmap VO 的 CPU/内存对比。

## P1a-3 — 纯视觉寻线 v1.0 待决策（`docs/design/vision_line_follow_v1.md`）

设计已出（草案，未改代码），实现前必须先拍板下面三件事；未定之前 M1 不开工。

- [ ] **赛道形态：一条可见标线（`guide_line`）还是左右边界（`track_corridor`）？**
  为什么卡着：这条决定 v1.0 主路径是几何响应（黑帽/顶帽/色度）还是"左右边界成对提取"；
  两者都没有时几何主路径不成立，必须改学习分割为主，§4.5 与里程碑要重写。
  决定了会影响：`line_mode` 默认值、扫描线取峰逻辑、`track_width` 是否参与置信度、
  M1 的录数口径与验收指标。
  判过：现场录 50 帧确认形态，结论写进设计 §11 Q1 并回填参数。
  依据：`docs/design/vision_line_follow_v1.md` §4.5.1、§11 Q1。

- [ ] **检测节点是否允许部署到 orin**（源码仍只在本机改，`~/rak` 只是部署副本）
  为什么卡着：1280×720 bgr8@30 = 663 Mbit/s、mono8 掩码 = 221 Mbit/s，都超过
  97 Mbit/s 链路；只有把 `line_detect_node` 放 orin、跨机只传约 100 B 的
  `LineObservation`，带宽预算才成立。否则要么本机收图（链路直接超），
  要么把控制也搬到 orin（偏离"上层在本机"）。
  决定了会影响：部署清单、orin 内存/CPU 预算、调试流 `debug_image` 是否允许、
  M1/M2 的联调方式。
  判过：确认 orin 可新增一个常驻节点（RSS < 200 MB、CPU < 25% 单核），
  写入设计 §3.1 并更新部署清单。
  依据：`docs/design/vision_line_follow_v1.md` §3.1、§3.3、§9.5。

- [ ] **外参标定是否执行**（卷尺 + 地面 5 个控制点，约 30 分钟）
  为什么卡着：单目 IPM 的比例完全依赖 `height/pitch/roll/x/y`；没标定时 BEV 的
  1 m 不是 1 m，横向误差与曲率都不可信（俯仰 0.5° 在 2 m 处就是 1.7 cm）。
  设计规定 `calibrated=false` 时节点拒绝启动，不留"看起来合理"的默认值。
  决定了会影响：M1 离线回放能否给出定量结论、M3 真机横向 5 cm 验收是否有意义、
  以及"先跑起来再补标定"这条捷径是否被允许。
  判过：按 §8.2 完成 5 点标定，`calibrated: true` + 日期 + 方法 + 重投影 RMS < 1.5 cm
  写入 `data/calib/front_camera_extrinsics.yaml`。
  依据：`docs/design/vision_line_follow_v1.md` §4.3、§8.2、§11 Q2。

## P1b — 挡住定量结论

- [ ] **轮径 + lx/ly 实测回填 URDF**（麦轮正解的前提）
  依据：`docs/reference/firmware_capability_spec.md` §4。

- [ ] **EKF 协方差标定**：Allan 静态噪声 + 直行 2 m 回归，回填 `ekf.yaml`
  依据：`docs/adr/0023-time-base-and-alignment.md`、`docs/status/next_2026-10-30.md` §6。
  工具：`scripts/calibrate_imu.py`。直行横向偏移 > 5 cm 时**先查轮径/轮距与零偏**，
  不要直接调融合参数。

- [ ] **时间基准实测记录落盘**（静止 30–60 min，记 `imu0_covariance[vyaw]`）
  依据：ADR-0023；记录写在 `docs/reference/perf/`（新文件：`timebase.md`）。

- [ ] **pitch/roll 符号约定现场核对**（核对前 Jetson 不消费 `0x2031`/`0x2032`）
  依据：`docs/reference/canopen_bitfields.md` §7。

- [ ] **`/scan` 跨机不可见**
  为什么卡着：lidar 启动时没带 `ROS_DOMAIN_ID=47` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
  + 钉 `wlP1p1s0` 的 `CYCLONEDDS_URI`，本机发现不到 writer
  （orin 本地 `/scan` 10 Hz / 525 线正常）。
  决定了会影响：上层能否在本机直接消费雷达（Nav2 代价层、录包口径、跨机诊断）。
  判过：本机 `ros2 topic hz /scan` 稳定 ~10 Hz，且 lidar 启动脚本统一带上述三项环境。

- [ ] **orin 缺 `osqp`**
  为什么卡着：`rak_wbc` 声明了依赖但部署未装，`wbc_node` configure 直接
  `ModuleNotFoundError`；当前靠 `~/.wbc-venv`（`--system-site-packages` + `osqp==1.1.3`）绕过。
  决定了会影响：wbc 能否按正常部署流程启动（venv 不是长期方案，且 aarch64 可得性未核）。
  判过：rosdep/部署清单包含 `osqp`，裸 `ros2 launch rak_wbc wbc.launch.py` 可 configure。

- [ ] **`rak_mcp` 未部署**
  为什么卡着：源码在，但 orin 与开发机都没有包/进程/配置，MCP 工具面无法真实调用。
  决定了会影响：验收第 7 项（工具真实调用生效）无法开展，上层只能走 ros2 CLI。
  判过：MCP server 可启动并被客户端列出工具，且一次限速内真实调用生效并留证。

- [ ] **启动所有权竞态**
  为什么卡着：`sensors.launch.py`（`publish_robot_tf:=true`）发布全车 URDF，
  `controller_manager` 以 `/robot_description` 话题为准 → 加载 `rak_arm`；
  臂板不在线时 `joint_state_broadcaster` 被连带停用。多条线并行时还会互相
  SIGKILL 对方的 ROS 栈（2026-11-16 实测 exit -9）。
  决定了会影响：并行联调能否可重复，以及 `slaves:=chassis` 是否真能裁剪。
  下一步：一条命令级单写者编排（清残留 → 只允许一个 URDF 发布者 → 起栈 → 校验控制器），
  并加 owner 标记避免跨线互杀。

## P2a — 板载 IMU 姿态（下位机 `~/vehical_rak_motion`）

- [ ] **D1 互补滤波 pitch/roll**：陀螺 X/Y 已在 14 字节块内（`app_imu.c:235-245`），
  加 ~25 行；零偏需顺带标 X/Y。判据：0.5 m/s² 加速与静止对比 ≤1°。
- [ ] **D2 `imu_evt` 倾斜有效性位**：高加速度/冲击期置位，主机据此决定是否用姿态修正。
- [ ] **D3 现场符号/零位核对**：水平→0°±0.5°，头下压 10°→pitch ∓10°，左抬 10°→roll ±10°。
  核对前 Jetson 不得消费 `0x2031`/`0x2032`（`canopen_bitfields.md` §7）。
  结论回填该文档 + 固件注释。

## P2b — 质量升级

- [ ] **底盘控制升级 C1–C5 按序推进**
  C1 静摩擦前馈机制已随 R76 落地，增益重整定等台架首测。
  依据：`docs/design/chassis_control.md` §6。

- [ ] **抗打滑三级接入的 σ 标定与 L3 gate 重接线**
  代码完成（L1 残差进 `/rak/metrics`、L3 gate 默认关、L2 静态回退）。
  架空期真打滑不可复现，落地期做。
  依据：`docs/reference/firmware_capability_spec.md` §7.2。

- [ ] **Nav2 `vx_min` 最低轮速钳位复核**（MPPI `vx_min` -0.10 → +0.02，basis 是推断）
  依据：`docs/design/chassis_control.md` §6。

- [ ] **zram/swap 取舍实测**（4 GB 板上是帮忙还是添乱，无实测不下结论）
  量 `si/so`、压缩比、真实负载下 p99 延迟。依据：`docs/reference/machine_baseline.md`。

- [ ] **`rak_metrics` 低速平坦样本占比**：Orin 上 `colcon` 编译后跑一次，留数据。
  依据：`docs/design/chassis_control.md` §6。

- [ ] **整机 5V/3A（15 W）供电预算不足**
  为什么卡着：USB 外设申报合计约 3.59 A > 3 A，余量为负；目前只有采样手段，
  还没有一次复位前后的完整证据链。
  决定了会影响：能否排除欠压导致的 Orin 反复重启，以及外设是否必须改独立 5V 供电。
  下一步：已落地 journald 持久化 + 50 ms 采样脚本（`/var/log/pwr/pwr.csv`），
  下次复位后看断点前 40 行区分欠压/断电；串口 `ttyTCU0` 抓 reset reason 待接线。

## P3 — 远期（前面全绿再动）

- [ ] **行为层 ADR-0025 阶段 1**：`behaviortree_cpp` 可用才开工；`rak_task_executor`
  加 `engine:=sm|bt`，**默认 `sm`**，验收要求两种引擎的 `/task/status` 序列逐字段相同。
  依据：`docs/adr/0025-behavior-layer-bt-and-smacc2.md`、
  `docs/status/next_2026-10-30.md` §9。

- [ ] **`docs/design/brain_arm_tools.md` 落 ADR 后再实现**（当前只是草案）。

## 文档与规范（持续）

- [x] 文档工程按 Diátaxis 重组 + 索引 + 状态行 + 门禁（2026-11-14）
  索引与写作规则：`docs/README.md`；门禁：`python3 scripts/check_docs.py`。
- [x] AGENTS.md 成为唯一代理规则文件，CLAUDE.md 删除（2026-11-14）。
- [x] 机械臂仓改名 `vehical_rak_edge_ai` → `vehical_rak_arm`（GitHub + 本地，2026-11-14）。
- [ ] 把 `python3 scripts/check_docs.py` 接进 `scripts/build_workspace.py` 的收尾检查，
  让文档门禁与构建门禁同批跑。
- [ ] 清理仍然悬空的文档引用（`docs/CAN-BRINGUP-2026-10-30.md` 从未存在过；
  含该引用的文件改成指向真实文档）。
- [ ] 每季度核对一次 `docs/README.md` 的"最后更新"列，过期的改成 `已过期` 并考虑移入
  `docs/archive/`。
- [ ] 项目 skills 更新：`cd ~/rak && npx skills@latest update -p -y`，改动进
  `skills-lock.json` 一起提交。
