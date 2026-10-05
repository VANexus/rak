# ADR-0006: 3D 避障用 STVL，SLAM 保留 slam_toolbox（3D 来源是深度相机，不是雷达）

- **状态**: 已接受（选型）；参数与实测 **待闭合**
- **日期**: 2026-10-05
- **相关**: `src/rak_navigation/config/nav2_params.yaml` · `src/rak_navigation/config/slam_toolbox.yaml` · `src/rak_sensor_bringup/config/lslidar_n10.yaml` · ADR-0002 · ADR-0004 · `TODO.md` P1-3/P1-4

---

## 背景

**传感器事实** `[已核实]`：

- 雷达是 **WHEELTEC LSLIDAR N10P —— 2D 单线雷达**（`lslidar_n10.yaml`：N10_P，108 B @ 460800）。
- 深度相机 Orbbec Astra + USB 双目。
- 本机**没有 3D 雷达**。

这个事实决定了两件事：

1. **3D 障碍信息只能来自深度相机**，不能来自雷达。
2. **FAST-LIO2 / LIO-SAM 这类 LiDAR-inertial 3D SLAM 不适用**（它们需要 3D 雷达）。
   当前 `slam_toolbox`（2D）是**与传感器匹配的正确选择**，不是权宜之计。

Nav2 侧现状 `[已核实]`：`controller_server` 用 `nav2_mppi_controller` + `motion_model: "Omni"`；
`max_velocity: [0.25, 0.15, 0.5]` 为占位值。

---

## 决策

### D1. 3D 避障层 = **STVL**（`spatio_temporal_voxel_layer`）

用 STVL 作为 Nav2 costmap 的 3D 层，输入是**深度相机点云**。

- **纯 CPU**（PCL 稀疏体素 + 时间衰减），**不需要 CUDA**。
- Nav2 文档口径：CPU 占用约为 `voxel_layer` 的 **1/2**，且带时间衰减（更适合动态环境）。
- 这是 4 GB 机型上**唯一不需要和感知栈抢显存的 3D 方案**。

### D2. SLAM 保留 `slam_toolbox`（2D）

- 与 2D 雷达匹配；`docs/architecture.md` §5 的"同一 TF 子坐标系只能有一个权威发布者"
  已由 ADR-0002 的 EKF 方案承接。
- **不引入 FAST-LIO2/LIO-SAM**：本机没有 3D 雷达。
- 若将来加装 3D 雷达，再单独评估 —— 那是硬件变更，不是参数调整。

### D3. `nvblox_nav2` 作为**备选**，不作为首选

Isaac ROS 4.6.0+ 才加入 JetPack 7.2 / Orin 支持（此前 4.0 仅 Thor/JP7）。
在 4 GB 上它是**显存紧张**的工作负载，且需要同时把 MoveIt2/SLAM 外移（见 ADR-0004 D3）
才有余量。**先用 STVL 把功能跑通**，nvblox 只在 STVL 被证明不够时再评估。

---

## 理由

1. **传感器匹配**：2D 雷达 + 深度相机 → 2D SLAM + 深度驱动的 3D 避障层，各用其长。
2. **内存是硬约束**（ADR-0004）：STVL 不占显存，nvblox 占；在 4 GB 上这个差别是决定性的。
3. **STVL 是 Nav2 官方教程覆盖的路径**，集成成本低，不需要引入 Isaac ROS 容器
   （Isaac ROS 容器在 4 GB 上构建会 OOM，已有公开报告）。
4. **不动 SLAM**：换 SLAM 会牵动 `odom_source` 接缝（ADR-0002）与 Nav2 参数，
   在传感器不变的前提下没有收益。

---

## 后果

### 正面

- 3D 避障**不与感知栈抢显存**，可与 DeepStream/Nav2/SLAM 共存（在 ADR-0004 D4 的天花板内）。
- 时间衰减对动态障碍友好（人走过留下的体素会自动消失）。
- 不需要 Isaac ROS 容器，部署面更小。

### 负面（代价）

- **CPU 消耗**：STVL 虽然比 `voxel_layer` 省一半，仍是纯 CPU 体素运算，会与
  `slam_toolbox`、Nav2 争 A78AE 核心。分辨率/体素尺寸必须实测调。
- **深度相机的固有短板**：Astra 的有效距离、视场角、对反光/透明/强光表面的失效，
  都会直接变成"看不见障碍"。**2D 雷达仍是主要的安全层**，3D 是补充。
- **需要外参标定**：深度相机相对 `base_link` 的外参错了，3D 障碍就会投到错误位置
  （`docs/architecture.md` §7 已列"硬件 TF"为缺口）。
- 深度点云与雷达点云的**时间同步**需要处理，否则快速转向时出现"鬼影障碍"。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| `nvblox_nav2` 作首选 | 4 GB 上显存紧张；需 Isaac ROS 4.6.0+ 且要先把 MoveIt2/SLAM 外移；先用 CPU 方案跑通更划算 |
| Nav2 `voxel_layer` | CPU 占用约为 STVL 的 2 倍（Nav2 官方口径），在 A78AE 上更紧张 |
| `obstacle_layer`（2D 光线投射） | 只能表达 2D 障碍，**看不到桌腿之外的悬空/低矮障碍**，正是要解决的问题 |
| 换 FAST-LIO2 / LIO-SAM | **需要 3D 雷达**，本机只有 2D N10P |
| 换 RTAB-Map 做 RGB-D SLAM | 4 GB 上边际（500 MB–1.5 GB+，需严格限内存）；且当前 2D 雷达 + slam_toolbox 已够 |

---

## 待闭合项

1. **STVL 未安装**（已核实 `spatio_temporal_voxel_layer` 缺失）。
2. **深度相机外参未标定**（相机 → `base_link`），3D 障碍会投错位置。
3. **深度-彩色对齐未做**（`docs/architecture.md` §7）。
4. **体素尺寸/衰减时间/分辨率需实测**，不能抄默认值。
5. **`max_velocity: [0.25, 0.15, 0.5]` 是占位值**（`nav2_params.yaml:171` 自注 simulation-only），
   按 ADR-0007 迁移后由 `mecanum_drive_controller` 的限幅参数承担。
6. **collision monitor 当前"不介入"**：开 Omni + 3D 层后，横移路径的安全边界要重新评估
   （麦轮横移时车体扫过面积与前进不同）。
7. **`slam_toolbox` 的 `max_laser_range: 15.0`** 是否匹配 N10P 实际量程，需核对。

---

## 参考

- Nav2 + STVL 官方教程：<https://docs.nav2.org/jazzy/tutorials/general_tutorials/navigation2_with_stvl/>
- Isaac ROS nvblox：<https://nvidia-isaac.github.io/nvblox>
- Isaac ROS 对 JP 7.2 / Orin 的支持讨论：<https://forums.developer.nvidia.com/t/eta-for-isaac-ros-release-for-jp-7-2-on-orin-devices/372134/8>
- `src/rak_sensor_bringup/config/lslidar_n10.yaml`（N10P 是 2D 雷达的实测依据）
- 本文结论由三路研究中的"DeepStream/Holoscan + 融合"专项得出（2026-10-05）
