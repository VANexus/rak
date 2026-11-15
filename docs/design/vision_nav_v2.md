# 4D 融合视觉导航 v2.0 —— 全传感器融合 + 时序体素代价地图 + 全向状态格寻路

> 状态：**草案已落地为 ADR-0027**（代码在 `src/rak_bringup`，门禁全绿；真机未验证）

- **日期**: 2026-11-15 · **决策**: [ADR-0027](../adr/0027-4d-fusion-navigation.md)
- **目标**: 把单目/双目/深度相机/2D 雷达/IMU/轮式里程计统一进一条状态估计与一张可衰减的
  三维代价地图（4D = 三维空间 × 时间），用原生全向规划器出高质量路径。
- **接续**: `vision_nav_v1.md`（v1.0 纯前视单目，已在跑）；v1 的 TwinLiteNet 路面分割、
  RTAB-Map 定位、MPPI 权重全部沿用，本文只做**加法**。
- **关联**: ADR-0002（EKF 两层结构）· ADR-0015（一帧一发布者）· ADR-0020（生命周期/运动许可）·
  `docs/reference/firmware_capability_spec.md`（轮式里程计职责与打滑）

## 1. 分层与归属

```
sensors.launch.py            lidar(由 slam.launch.py 起) + 深度相机 + /camera/depth/points + IMU
vio.launch.py                rtabmap_odom rgbd/stereo -> /odometry/vio     (publish_tf: false)
nav2.launch.py
  slam.launch.py             rf2o -> /odom · robot_localization EKF(ekf_4d.yaml) · slam_toolbox
  Nav2 servers               controller(MPPI Omni) · planner(SmacLattice omni) · behaviors · bt
  4d overlay(nav2_4d.yaml)   local/global costmap += rtabmap VoxelLayer(decay)
```

TF 归属不变（ADR-0002/0015）：EKF 独占 `odom -> base_footprint`，slam_toolbox 独占
`map -> odom`，`robot_state_publisher` 管机体树，深度相机驱动管光学帧。
`allow_motion` 仍是唯一运动许可；本栈起来不动任何执行器。

## 2. 传感器 → 库 映射（全部现成件）

| 传感器 | 现有驱动/话题 | 融合去哪个库 | 角色 |
|---|---|---|---|
| 前视单目 UVC | `usb_cam` → `/image_raw` | `rak_road_perception`（TwinLiteNet TRT）+ rtabmap 定位 | 可行驶区域、视觉重定位 |
| 双目 SBS | `rak_stereo_driver` → `eye/left|right` | `stereo_image_proc`（apt，需标定）→ `/stereo/points` | 稠密深度（未标定前空载） |
| Astra / Gemini 335L | Orbbec 驱动 → `/camera/depth/image_raw` | `depth_image_proc/point_cloud_xyz` → `/camera/depth/points` | 近场稠密体素源 |
| N10_P 2D 雷达 | `lslidar` → `/scan` | rf2o（里程计）+ 体素层（障碍） | 360° 平面、唯一长距源 |
| LSM6DSRTR IMU | `rak_imu_driver` → `/imu/data_raw` | `robot_localization`（imu0, vyaw） | 航向角速率权威 |
| 轮式（麦轮） | `mecanum_drive_controller` → `/odometry/wheel` | EKF `odom0`（差分；yaw 排除） | 平移增量、抗打滑 |

**VIO 的诚实口径**：`rtabmap_odom` 出视觉增量 + EKF 融陀螺 = **松耦合 VIO**。
紧耦合（VINS/OpenVINS/cuVSLAM）都在 ADR-0027「被否决/待闭合」里，没有偷偷上。

## 3. 4D 代价层

四维中的第四维 = **时间衰减**：

- 局部层 `decay: 5.0 s`、`z_resolution: 0.05`、`z_voxels: 16`
- 全局层 `decay: 0.0`（持久）、`z_resolution: 0.1`、`z_voxels: 12`
- 观察源三路：`scan`（`sensor_frame: laser_link`）、`depth_cloud`、`stereo_cloud`
- 全地形口径：`min_obstacle_height=0.06` 的深度点云源把 ≤6 cm 台阶视为可翻越；
  雷达源必须 `min_obstacle_height=0.0`（z=0 平面，任何正值都会把点全滤掉，实测踩过）

为什么不是 STVL：见 ADR-0027「理由」与本文 §6 实测记录。

## 4. 寻路与控制的"高质量"来自哪

1. **规划**：`SmacPlannerLattice` + omni 原语（5 cm 分辨率、0.5 m 转弯半径、16 朝向），
   路径本身允许横移，不是"差速车假装全向"。
2. **控制**：MPPI `motion_model: Omni`，critic 权重沿用 v1.0 实测（`TwirlingCritic` 10、
   `PathAlignCritic` 10、`vy_std` 0.08）；`vx_min` 0.02 避开固件 stick-slip 区。
3. **地形偏好**：体素层给障碍（3D + 时间），`inflation_radius 0.5` 给安全裕度；
   路面分割在 v1.1 里只做减速（`PolygonSlow`），本文不改它的语义。

## 5. 怎么跑

```bash
colcon build --packages-select rak_bringup && source install/local_setup.bash

# 全栈（深度相机 + 雷达 + IMU + EKF 四路 + VIO + Nav2 4D）
ros2 launch rak_bringup 4d_nav.launch.py
ros2 launch rak_bringup 4d_nav.launch.py vio:=none            # 不要视觉里程计
ros2 launch rak_bringup 4d_nav.launch.py map_yaml:=/abs/map.yaml
ros2 launch rak_bringup 4d_nav.launch.py motion_backend:=none # 外层 bringup 收 /cmd_vel

# 门禁（不需要硬件）
python3 -m pytest src/rak_bringup/test/test_4d_nav_config.py -q      # 8 项配置契约
python3 scripts/verify_4d_fusion.py                                  # 4 道运行时门
python3 scripts/verify_4d_fusion.py --only costmap                   # 单跑体素层门
python3 scripts/verify_4d_fusion.py --only vio                       # 单跑视觉里程计门
python3 scripts/verify_4d_fusion.py --only path                      # 单跑"真出一条路"门
```

## 6. 实测记录（2026-11-15，本机 Orin Nano 4 GB / L4T R39.2 / CUDA 13.2）

`python3 scripts/verify_4d_fusion.py` → **ALL GATES PASSED**

| 门 | 判据 | 结果 |
|---|---|---|
| G1 | EKF 四路输入 + 每路协方差 + 局部体素层带 decay + 全局保留 static_layer | PASS |
| G2 | 喂 4 路后 `/odometry/filtered` 出现，且 `odom->base_footprint` 由 EKF 发布 | PASS |
| G3 | 合成 `/scan` + `/camera/depth/points` → `/local_costmap/costmap` 出现标记 | PASS，`marked_cells=1549`，`max=100` |
| G4 | `planner_server` configure 成功且日志为 `Using motion model: State Lattice` + omni 原语 | PASS |
| G5 | `rtabmap_odom rgbd_odometry` 吃合成 RGB-D，`/odometry/vio` 出 `odom -> base_footprint` 消息 | PASS |
| G6 | 同一 planner + 全 Free `/map`，`ComputePathToPose` 返回 `SUCCEEDED` 且路径 202 点 | PASS |

调试过程中被实测钉住的四个坑（都在代码注释里留了痕）：

1. **rcl 参数解析**：协方差必须是**单行 flat 225 元素**且同列表元素类型一致（整数 `0` 会让
   整文件解析失败；多行块序列报 `Sequences cannot be key`）。
2. **STVL 不标记**：初始化/激活/订阅都正常，合成数据打进去 `max_cost=0`；
   同场景换 `rtabmap_costmap_plugins::VoxelLayer` 立刻 1549 格。
3. **STVL/nav2 平面雷达高度**：`min_obstacle_height: 0.02` 会把 z=0 的雷达点全部滤掉。
4. **门的顺序**：先起传感器再起 Nav2 才标记；反过来（costmap 先激活）本机复现 0 标记。
   真实 bringup 就是"驱动先起"，门禁按真实顺序跑。
5. **rtabmap 参数声明类型**：`Odom/*`、`Vis/*`、`GFTT/*` 在 0.23.13 里声明成 **string**，
   从 yaml 传 double/bool 会抛 `InvalidParameterTypeException` 直接把节点打死
   （`GFTT/MinDistance`、`Odom/AlignWithGround` 都踩过）。`vio_rtabmap.yaml` 因此只留
   接线必需项。

## 7. 未验证项 / 下一步

| 项 | 判据 | 依据 |
|---|---|---|
| 真机深度相机 + 雷达进体素层 | 两路 `topic hz` 与 costmap 标记同时成立 | ADR-0027 待闭合 1 |
| EKF 协方差标定 | Allan + 直行 2 m 回归回填 `ekf_4d.yaml` | ADR-0002、TODO P1b |
| 双目标定 → `/stereo/points` | `stereo_image_proc` 出非空点云 | ADR-0027 待闭合 3 |
| 全栈内存 | 组件 RSS 合计 + 最低可用内存 < 3.4 GB | `vision_nav_v1.md` §5A.10 口径 |
| STVL 重测 | 同门禁通过则换回（稀疏网格省内存） | ADR-0027 待闭合 4 |
| cuVSLAM 对照 | CPU/内存与 rtabmap VO 对比 | ADR-0027 待闭合 6 |
| 底盘真机运动 | `allow_motion:=true` 后 MPPI 直道 10 m 横向偏差 < 5 cm | `hardware_acceptance.md` |

## 8. 与 v1.0 的兼容性

- `nav2_params.yaml` **未改动**；4D 全部走 `params_overlay`，v1 命令行为不变。
- `nav2.launch.py` 只新增三个默认空参（`ekf_params` / `params_overlay` / `lattice_filepath`），
  默认值等价于旧行为。
- `slam.launch.py` 的 `ekf_params` 默认仍指向 `config/ekf.yaml`（v1：rf2o + IMU）。
- 新增包依赖：`ros-jazzy-rtabmap-costmap-plugins`、`ros-jazzy-stereo-image-proc`
  （未接线）、`ros-jazzy-grid-map-*`（v2.1 地形层备用）。STVL 装了但当前未启用。
