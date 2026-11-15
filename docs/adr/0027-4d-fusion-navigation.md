# ADR-0027: 4D 融合视觉导航 v2.0 —— 多源状态估计 + 时序体素代价地图 + 全向状态格规划

- 状态: 已接受（**软件门禁全绿，真机未验证**；见「待闭合项」）
- 日期: 2026-11-15
- 相关: `src/rak_bringup/launch/4d_nav.launch.py` · `config/ekf_4d.yaml` ·
  `config/nav2_4d.yaml` · `config/vio_rtabmap.yaml` · `launch/vio.launch.py` ·
  `test/test_4d_nav_config.py` · `scripts/verify_4d_fusion.py` ·
  `scripts/fake_4d_sensors.py` · `docs/design/vision_nav_v2.md`（设计依据）·
  `docs/design/vision_nav_v1.md`（v1.0 纯视觉，本文接续）· ADR-0002（EKF 两层结构）·
  ADR-0020（生命周期与运动许可）

> 编号说明：ADR-0026 已被组装层拆分占用，本篇顺延为 0027。

## 背景

v1.0（`vision_nav_v1.md`）是**纯前视单目**：TwinLiteNet 分割 + 固定地图定位 + Nav2 MPPI。
它刻意放弃了运行时雷达与深度相机，也只有一个 2D 障碍层——锥桶被搬走，代价地图要等到
下一次 raytrace 才放开，斜坡/台阶这类"能过但不好过"的地形没有表达。

v2.0 要的是**4D（三维空间 × 时间）融合**：把手上所有传感器（前视单目、双目、Astra/Gemini
深度相机、N10_P 雷达、SPI IMU、CANopen 轮式里程计）塞进同一条状态估计与同一张可衰减的
三维代价地图，并用**原生全向**规划器出高质量路径。硬约束：

1. **4 GB 统一内存**：现有全栈 2.0–3.0 GB 已压线（`vision_nav_v1.md` §5A.10 实测），
   任何融合方案必须先过内存这一关。
2. **不造轮子**：用户明确要求复用现成工程/库；本仓只有"接线 + 参数 + 门禁"是新代码。
3. **不放松安全**：`allow_motion` 仍是唯一运动许可，激活状态不等于能运动（ADR-0020 §6）。
4. 板子是 JetPack 7.2 / L4T R39.2 / CUDA 13.2 / ROS 2 Jazzy；apt 之外的东西都要单独论证。

## 决策

1. **状态估计**：`robot_localization` EKF 吃四路——轮式 `/odometry/wheel`、rf2o `/odom`、
   视觉 `/odometry/vio`、IMU `/imu/data_raw`（`config/ekf_4d.yaml`）。轮式与视觉用**差分增量**，
   绝对位姿只留 rf2o 一路（ADR-0002 §约束 4）。EKF 仍是 `odom -> base_footprint` 的唯一发布者。
2. **VIO**：视觉侧用 **`rtabmap_odom`**（apt 0.23.13，`rgbd_odometry` / `stereo_odometry`）出
   `/odometry/vio`，`publish_tf: false`；陀螺在 EKF 层融合 = **松耦合 VIO**，不自己写滤波器。
3. **4D 代价层**：`rtabmap_costmap_plugins::VoxelLayer`（apt `ros-jazzy-rtabmap-costmap-plugins`）
   进 Nav2 local + global costmap，三路观察源：`/scan`、`/camera/depth/points`、`/stereo/points`；
   局部层 `decay: 5.0`（秒）给时间维，全局层 `decay: 0.0`（持久）。
4. **规划**：全局规划器换 `nav2_smac_planner::SmacPlannerLattice`，lattice 用上游自带的
   `sample_primitives/5cm_resolution/0.5m_turning_radius/omni/output.json`（全向原语），
   路径由 launch 用 `get_package_share_directory` 解析，参数文件不写死 distro 路径。
5. **控制不动**：MPPI（`motion_model: Omni`）与现有 critic 权重沿用 v1.0 实测值。
6. **入口**：`ros2 launch rak_bringup 4d_nav.launch.py`（`vio:=rgbd|stereo|none`、
   `camera:=gemini335l|astra`、`motion_backend:=sim|stub|none`）。`nav2.launch.py` 新增
   `ekf_params` / `params_overlay` / `lattice_filepath` 三个透传口子，v1 行为不变。

## 理由

- **EKF 四路**：v1 只有 rf2o + IMU，一旦雷达被遮挡或地面退化就只剩陀螺积分。轮式补平移、
  视觉补弱纹理以外的场景，四路互为备份；全部是 ADR-0002 早就画好的槽位（F1→F2→F3），
  这次只是把 F3 接上。
- **rtabmap 而不是自研或 cuVSLAM**：apt 已装 0.23.13，与 rtabmap SLAM/定位共用同一份特征
  引擎；cuVSLAM（Isaac ROS 4.6）虽官方支持 JP7.2 + Jazzy，但要走 NVIDIA apt 仓库 + NITROS
  运行时，4 GB 板子上是额外常驻；留作 §「待闭合项」的对照实验。
- **体素层选 rtabmap 版**：2026-11-15 同一份合成 `/scan` + `/camera/depth/points` 打进去，
  STVL 2.5.5 初始化、激活、订阅全绿但**一格都没标**（max_cost=0）；`rtabmap_costmap_plugins`
  与 nav2 自带 VoxelLayer 立刻标出 1549 格（max=100）。能标的那个才有意义；
  STVL 的 OpenVDB 稀疏网格仍是大范围地图的更优解，列进待闭合项重测。
- **SmacPlannerLattice omni**：1.3.13 的 `SmacPlannerHybrid` 只接受 2D/DUBIN/REEDS_SHEPP/
  STATE_LATTICE（`constants.hpp::fromString` 无 OMNI 分支），写 `motion_model: "Omni"` 会
  configure 失败；而 lattice 的 omni 原语是随包发布的，本机 configure 实测
  `Using motion model: State Lattice` 通过。麦轮全向车用全向原语规划，横移才进得了路径。
- **不引入点云预处理节点**：`depth_image_proc/point_cloud_xyz`（apt）已把深度图变成点云，
  `stereo_image_proc`（apt）在有标定后同理；空隙由体素层的 `min/max_obstacle_height` 处理。

## 后果

**正面**

- 一张命令起 4D 全栈：`4d_nav.launch.py`，sensors/VIO/EKF/Nav2 三段复用既有入口。
- 时序衰减让"看到过、现在没了"的障碍自动退场，不再依赖 raytrace 覆盖。
- 规划器原生全向；麦轮横移不再被 2D 差速格栅"假装直走"。
- 门禁可复现：`pytest src/rak_bringup/test/test_4d_nav_config.py`（8 项，离线）+
  `python3 scripts/verify_4d_fusion.py`（4 道门，本机全绿，不需要硬件）。

**负面（代价，必须写）**

- **多一路订阅就多一份 CPU**：`/camera/depth/points` 在 640×480 下约 1–2 MB/帧，
  体素层在 Orin Nano 上是可测的开销；`publish_voxel_map` 默认关闭省 CPU。
- **EKF 四路互相打架的风险真实存在**：协方差全部是占位值（`TODO(measured)`），
  真机必须按干扰观测/Allan 重标，否则滤波器会相信打滑的轮式或漂移的 VO。
- **rtabmap VoxelLayer 的上游维护弱于 STVL**；`decay` 语义是它自己的扩展，
  版本升级要重跑门禁。
- 深度相机与双目点云**目前没有标定**：`/stereo/points` 在标定文件出现前不会有发布者，
  该路源只是"接线就绪"，不是"已工作"。
- 全局 costmap 加了体素层后内存比 v1 高（0.1 m 分辨率、12 层），4 GB 板子上限仍在测量。
- **rtabmap 0.23.13 的参数声明类型是个坑**：`Odom/*` / `Vis/*` / `GFTT/*` 被声明成
  string，从 yaml 覆盖成 double/bool 会让节点启动即抛 `InvalidParameterTypeException`。
  `config/vio_rtabmap.yaml` 只能保留接线必需项，算法参数调不了——这是选它的真实代价。

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| `spatio_temporal_voxel_layer`（STVL 2.5.5） | 本机实测不标记（见上）；**不是**否决它的设计，是等它在本栈上重测通过再换 |
| `elevation_mapping_cupy` | ROS 2 分支要 CuPy，CuPy 官方轮子跟 CUDA 13 不配套；Jetson 4 GB 无此余量 |
| `nvblox` / `isaac_ros_nvblox` | Isaac ROS 4.6 确实支持 JP7.2 + Jazzy，但 ESDF/TSDF 需 GB 级显存；4 GB 板上与现有栈不共存 |
| Isaac ROS 其余件（cuVSLAM、image_proc、tensor_rt） | 走 NVIDIA apt 仓库 + `isaac-ros-cli`，没有 per-package deb；本仓 apt 优先，列为 v2.1 对照实验 |
| Holoscan SDK 5.0 做机器人中间件 | 5.0 取消了 GXF/Application/scheduler，**没有 ROS 2 bridge**；HoloHub 的 ROS 2 应用已转 legacy |
| DeepStream 9.2 接 ROS 2 | 无维护中的 ROS 2 桥（`ros2_deepstream` 类都已停更）；且 v1 实测 `nvv4l2decoder` 解 MJPG 失败、batch=6 仅 +15%（访存受限），加速解码对本机无收益 |
| OpenVINS / VINS-Fusion / stella_vslam / ORB-SLAM3 | 都不在 Jazzy rosdistro；要么无 ROS 2 分支，要么 GPL 传染 + Ceres/OpenCV 钉子；留作紧耦合 VIO 的对照 |
| `SmacPlannerHybrid` + `motion_model: "Omni"` | 1.3.13 不认该值，configure 直接失败 |
| 自研点云/占据融合节点 | 用户要求不造轮子；`depth_image_proc` + 体素层已覆盖 |

## 待闭合项

1. **真机传感器验证**：深度相机 / 雷达 / 双目当前都没插；`4d_nav.launch.py` 只在仿真与
   合成传感器下验过。首个真机判据 = `/camera/depth/points` 与 `/scan` 同时进体素层，
   `ros2 topic hz` 与 `/local_costmap/costmap` 标记同时成立。
2. **EKF 协方差标定**：Allan 噪声 + 直行 2 m 回归 + 打滑场景，回填 `ekf_4d.yaml`；
   在此之前任何"融合效果"结论都只是接线正确，不是精度达标。
3. **`/stereo/points` 需要双目标定**：`stereo_image_proc` 已 apt 安装但未接线；标定文件出现前
   该观察源保持空。
4. **STVL 重测**：本机为何不标记未查明（可能与其 OpenVDB 版本或 sub-voxel 过滤有关）；
   若通过则换回，拿它的稀疏网格换内存。
5. **内存实测**：全栈（深度相机 + 雷达 + EKF + VIO + Nav2 + 体素层）RSS 与最低可用内存；
   超过 3.4 GB 就砍全局体素层或降分辨率。
6. **`isaac_ros_visual_slam` 对照**：若 NVIDIA apt 仓库在当前镜像上可用，测 cuVSLAM 在
   Orin Nano 的 CPU/内存占用，与 rtabmap VO 对比后决定是否替换。
