# ADR-0002: 用 `robot_localization` EKF 做单一 `odom→base_link` 发布者，分阶段融合四路里程计

- **状态**: 已接受（架构）；参数与源权重 **待实测标定**
- **日期**: 2026-10-05
- **2026-10-05 更新（随 ADR-0007）**：轮式里程计的来源改为
  **`mecanum_drive_controller/odometry`**（由轮速/轮位反馈算出），
  不再是"MCU 回传的机体速度"。同时 **`mecanum_drive_controller` 必须设
  `enable_odom_tf: false`**，否则与本文的 EKF 抢发 `odom→base_link`。
  F407 板载 IMU 收窄为**轮级**自稳定，其数据**不进入**本 EKF
  （机体航向权威是 Jetson 的 LSM6DSRTR）。
- **相关**: `src/rak_navigation/launch/slam.launch.py` · `src/rak_navigation/launch/nav2.launch.py` · `src/rak_navigation/config/slam_toolbox.yaml` · `src/rak_base_driver` · `scripts/lsm6dsr-overlay/` · ADR-0007

---

## 背景

现状（`slam_toolbox.yaml` 第 3–7 行原文）：

> "This file deliberately mentions only the `odom` and `base_link` frame names and nothing
> about where odometry comes from. That is the whole point: **the robot currently has no
> wheel encoders and no IMU**, so `odom -> base_link` is [由 rf2o 提供]."

即当前 `odom→base_link` 由 **rf2o（2D 激光里程计）** 提供，`slam.launch.py` 暴露了
`odom_source:=rf2o|ekf|none` 的**接缝**，但 `docs/explanation/architecture.md` §7 明确：

> "当前 `odom_source:=ekf` 预留选项**不会自动启动 EKF**；仍需外部提供有效 TF。"

要融合的四路（用户目标）：

| 源 | 提供者 | 频率 | 现状 |
|---|---|---|---|
| **轮式里程计** | 底盘 F407（UART 遥测） | 100 Hz | [待实现] 固件不在本仓库 |
| **IMU 里程计** | LSM6DSRTR（40-pin SPI，IIO） | 最高 6.66 kHz | [待实现] overlay 已备好未应用 |
| **雷达里程计** | rf2o（N10P 2D） | ~10–20 Hz | [已实现] 当前唯一源 |
| **视觉里程计** | 双目 / Astra 深度 | 30 Hz | [待实现] 无 VO 节点 |

**为什么必须融合**（麦轮的固有特性，与 IK 放在哪一侧无关）：麦轮里程计在 **y（横移）方向很差** ——
辊子侧滑、地面摩擦各向异性使 `(vx, vy, wz)` 积分出的横移分量不可信。
单靠轮式里程计做 `odom→base_link` 会导致 Nav2 定位漂移，横移越多漂得越快。

---

## 决策

**引入 `robot_localization` 的 EKF 作为 `odom→base_link` 的唯一发布者，分阶段接入四个源。**

1. **单一权威发布者**：`odom→base_link` 只由 EKF 发布。rf2o 不再直接发 TF，改为
   发布带协方差的 `nav_msgs/Odometry` 作为 EKF 的输入之一。
   （`docs/explanation/architecture.md` §5 已有原则："同一个 TF 子坐标系只能有一个权威发布者"。）
2. **分阶段接入，每阶段都必须实测验证后再加下一个源**：

   | 阶段 | 融合内容 | 前置条件 | 验收 |
   |---|---|---|---|
   | F1 | 轮式 odom + IMU | 底盘 UART 遥测可用、IMU 驱动就绪 | 静止无漂移；直行 2 m 误差可测 |
   | F2 | + 雷达 odom（rf2o） | F1 稳定 | 横移后 y 漂移显著小于纯轮式 |
   | F3 | + 视觉 odom | VO 节点可用 | 长走廊/低特征场景鲁棒性提升 |

3. **两层定位，不合并成一个**：
   - `odom→base_link`：EKF（连续、高频、局部，允许漂移）
   - `map→odom`：SLAM（slam_toolbox，低频、全局、消除累积漂移）
   这是 ROS 的标准两层结构，不要试图用 EKF 同时解决全局定位。

4. **所有输入必须带真实协方差与时间戳**。`docs/explanation/architecture.md` §5 明确要求"感知输出需保留
   采集时间、坐标系、有效性和数据年龄"。**没有协方差的里程计不要塞进 EKF** ——
   默认协方差会让 EKF 信任错误的源。

### 关键配置约束（研究结论，2026-10-05）

这些是 `robot_localization` 在**麦轮**上最容易踩的坑，逐条都要在配置里体现：

1. **不要融合轮式里程计的 yaw（姿态或角速率都不要）。** 航向是最噪的派生量，
   误差随距离线性增长、随航向漂移**二次**增长。航向只由 **IMU 的 gyro-z** 提供。
2. **vy 必须融合，但要放大它的协方差。** 麦轮横移时辊子滑移对编码器**不可见**
   （IK 雅可比有零空间：某些轮速组合对应纯滑移、零机体运动），
   所以 **vy 是系统性地最差的通道**。让 vy 的协方差显著大于 vx（例如 0.5 vs 0.1），
   或用随 `|vy|`/加速度缩放的动态协方差。
3. **不要同时融合同一源的位置和速度**（双计）。每个源的 `*_config` 矩阵里只开一种。
4. **绝对 SLAM 位姿必须用 `differential: true`**（或用 `relative`），
   否则两个绝对源会在 EKF 里互相打架。**只允许一个源 `differential: false`。**
5. **共享 IMU 相关性**：若雷达/视觉里程计内部已经融合了同一个 IMU，
   再把它单独塞进 EKF 会**过权重**。要么只喂给里程计，要么接受这个相关性。
6. **IMU 时间戳必须来自内核采样时刻**，不能用"处理完成时间"。
   不同源速率不同时用 `message_filters::ApproximateTime` 对齐。
7. `two_d_mode: true`（地面机器人），丢弃 IMU 的 roll/pitch 噪声但保留 gyro-z。

---

## 理由

1. **四路源的可信度不同且互补**：轮式低频但连续（除横移）、IMU 高频但漂移、
   雷达/视觉绝对但慢且易受场景影响。EKF 正是为这种互补设计的。
2. **接缝已经存在**：`odom_source:=ekf` 已预留，改动是"填充"而不是"重构"。
3. **麦轮的固有缺陷需要外部源纠正** —— 辊子滑移对编码器不可见，IK 在上位机还是 MCU 都一样。
4. **`robot_localization` 是 ROS 2 的事实标准**，Jazzy 已装（`ros-jazzy-robot-localization`），
   不需要自研滤波器。

---

## 后果

### 正面

- 单一 TF 发布者，消除多源抢 TF 的竞态。
- 横移精度可显著改善（IMU 的 yaw + 雷达/视觉的绝对位置纠正轮式的 y 漂移）。
- 分阶段接入，每步可独立回滚（`odom_source:=rf2o` 是安全回退点）。

### 负面（代价）

- **调参成本高**。`robot_localization` 的协方差矩阵与 `*_config` 参数对结果影响很大，
  必须实测标定，不能抄默认值。
- **多一层延迟与故障点**。EKF 挂了整条 `odom→base_link` 就断了；需要 watchdog。
- **IMU 与轮式的坐标系/符号约定必须逐一对齐**（`docs/explanation/architecture.md` §5：ROS 是 x 前 y 左，
  旧 `SceneState` 是相机 x 右 z 前，**在适配层显式变换**）。符号错了 EKF 会发散而不是报错。
- **IMU 挂在 SPI 上**，与相机/雷达无争用，但 IIO→ROS 的桥接节点是新增的故障面。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 继续只用 rf2o，不融合 | 2D 激光里程计在开阔/长直场景会退化（特征少），且**无法提供横移分量**的观测 |
| 用轮式里程计直接发 `odom→base_link`，不加 IMU | 麦轮 y 漂移不可接受，且高频遥测丢帧会直接进 TF |
| 自己写 EKF/UKF | `robot_localization` 是成熟标准件；自研没有收益，只有维护成本 |
| 单层定位（EKF 直接出 `map→base_link`） | 会失去 SLAM 的全局闭环能力；两层结构是 ROS 标准，不应合并 |

---

## 待闭合项

1. **`odom_source:=ekf` 目前不会启动 EKF**（`docs/explanation/architecture.md` §7）。需要真正实现这个分支：
   启动 `robot_localization` 节点 + 加载配置 + 确保 rf2o 改为只发 Odometry 不发 TF。
2. **底盘遥测是否提供可用协方差未知**。`docs/explanation/architecture.md` §1："底盘有独立 IMU 做稳定，
   但是否提供 ROS 可消费的 IMU/协方差/时间戳还需对接。"
3. **IMU 的 IIO→ROS 桥接节点未实现**（见 `docs/tutorials/construction.md` §5）。
4. **视觉里程计节点不存在**。双目/Astra 都只有驱动，没有 VO 算法。
5. **坐标系与符号对齐未验证**：轮式、IMU、雷达三者的 x/y 方向与 yaw 正负必须逐一对齐，
   否则 EKF 发散。这是最容易出错且最难排查的一步。
6. **[需实测]** 全部协方差与源权重参数。

---

## 参考

- `src/rak_navigation/config/slam_toolbox.yaml`（"no wheel encoders and no IMU" 现状说明）
- `src/rak_navigation/launch/slam.launch.py`（`odom_source` 接缝）
- `docs/explanation/architecture.md` §5（时间/坐标系/TF 单发布者原则）、§7（EKF 未启动的缺口）
- ADR-0007（当前 ros2_control 架构；轮式里程计来源 = `mecanum_drive_controller/odometry`）
- 外部研究结论（`robot_localization` 双 EKF 架构、四源融合实践）—— 见本文档修订记录
