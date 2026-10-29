# ADR-0015: 全机坐标系分层 —— REP-120 body 树 + 无界 world 根 + 传感器/机械臂挂载帧

- **状态**: 已接受（2026-10-11）
- **日期**: 2026-10-11
- **相关**: `src/rak_description/urdf/rak_chassis.urdf.xacro`（全机静态骨架单一来源）·
  `src/rak_bringup/launch/slam.launch.py`（静态几何/里程发布者）·
  `src/rak_bringup/config/ekf.yaml` · `src/rak_bringup/config/nav2_params.yaml` ·
  ADR-0002（EKF 里程）· ADR-0007（ros2_control/单一运动写权）· ADR-0009（无 CAD 建模）·
  ADR-0011（WBC 基座模型）· ADR-0014（相机契约）
- **取代**: 无。**部分覆盖** `nav2_params.yaml` 旧约定"base_link 即 footprint 中心、无 base_footprint"。

---

## 背景

- 车体总架构确定（2026-10-11 用户实测）：**轮距（左右轮距）27 cm、轴距（前后轮距）25 cm**；
  装载为**单条纵中轴线**——雷达在前、机械臂在后，6 阵列麦克风在雷达正上方，深度相机在车头最前。
  目标是**无限开放世界小车**：会跑出单个房间/单张地图的边界。
- 现状（本 ADR 前）坐标系是**碎片拼出来的**，无单一权威：
  - `rak_chassis.urdf.xacro` 定义了 `base_footprint`，但 `nav2_params.yaml` 注释声明"无 URDF、无
    base_footprint，`base_link` 就是 footprint 中心"——两处直接冲突。
  - 传感器挂载靠 `slam.launch.py` 里的 `static_transform_publisher`（`base_link→laser_link`、
    `base_link→imu_link`），几何来源散落在 launch，且与 `chassis_bringup` 的
    `robot_state_publisher` 存在**同帧双发布**风险。
  - 顶层只有 `map`/`odom`，没有能表达"无界世界"的全局根系；`map` 同时被当全局系和局部系用。
- 工作空间已有分层硬约定（ADR-0013）：消息契约冻结、上层不感知传输/传感器后端；
  "**同一个 TF 子坐标系只能有一个权威发布者**"（`docs/architecture.md`）。

## 决策

1. **采用 REP-103 + REP-120**：右手系 `X 前 / Y 左 / Z 上`；`base_footprint` 恒在地面 (z=0) 作
   导航参考点，`base_link` 为车体中心（轮系几何中心），二者由固定竖直偏移 `base_height` 相连。
2. **TF 分层固定为**：
   `world → map → odom → base_footprint → base_link → {4 轮, laser_link, mic_array_link, camera_link, imu_link, arm_base_link}`。
3. **`world` 是无界全局锚点，作为根**。`map` 是当前局部地图（可重定位、允许跳变），
   `odom` 是连续里程（无跳变、缓慢漂移）。三者语义不得混用：导航参考、局部一致、全球持久各归其位。
4. **单一静态几何来源 = `rak_chassis.urdf.xacro`**，由 `robot_state_publisher` 发布
   `base_footprint→base_link` 及其下全部挂载帧（传感器 + 机械臂安装点均为 fixed joint）。
   **禁止在 launch 里为这些帧再放 `static_transform_publisher`**（同帧双发布）。
5. **发布者归属（每帧唯一）**：
   | 变换 | 唯一发布者 |
   |---|---|
   | `world → map` | 全局锚点节点（暂由静态恒等占位） |
   | `map → odom` | `slam_toolbox` |
   | `odom → base_footprint` | EKF（`odom_source:=rf2o` 时为 rf2o） |
   | `base_footprint → base_link` 及其下静态帧 | `robot_state_publisher`（载入全机 xacro） |
   | 机械臂 6 轴 + 夹爪（`arm_base_link→…→tool0`） | 关节接口就位后由 RSP + `joint_states` 提供 |
6. **实测几何入 URDF**：四轮 `(±0.125, ±0.135, 0)`；挂载帧 `laser_link` 前·中轴、
   `mic_array_link` 雷达上方、`camera_link` 车头最前、`imu_link`、`arm_base_link` 后·中轴。
   高度/前后精确距离以 `TODO(measured)` 占位，可经 xacro arg（`base_height`/`lidar_z`）覆盖。
7. **相机光学帧不由本仓定义**：`camera_*_optical_frame` 归 Orbbec 驱动发布，本仓只挂
   `camera_link`；`base_link→camera_link` 外参按 ADR-0014 实测，不得臆造。

## 理由

- **无限开放世界必须有独立于地图的全局根**。`map` 是 SLAM 的产物，会随重定位跳变、随会话重建；
  拿它当全球参考等于把"局部一致"和"全球持久"绑死。`world` 独立后，多张局部地图、跨房间
  拼接、户外 ECEF/UTM 锚定都有落点，`map` 只需对 `odom` 局部负责。
- **REP-120 拆开导航参考点与车体中心**。导航/代价地图关心的是地面投影（footprint），
  动力学/传感器关心的是车体中心（质心附近的旋转中心）。二者混为一体（旧 `nav2_params` 方案）
  会让 footprint 偏移在转向时变成系统性航向误差，且与 `mecanum` 的旋转中心语义纠缠。
- **几何单一来源 + 每帧唯一发布者**是 ADR-0013 分层原则在 TF 上的具体化：传感器换位置只改
  URDF 一处；换相机/雷达后端不改 TF 结构；launch 不再散落各传感器的硬编码偏移。
- **四轮几何实测入 URDF** 让 `robot_state_publisher`、MoveIt、`rak_wbc` 与可视化看到同一份
  轮距/轴距，避免"控制器参数一套、URDF 一套"的漂移。

## 后果

- **正面**：TF 树唯一、可推导、可视化自洽；导航参考点与车体中心解耦；无限世界有明确扩展点；
  传感器布局改动集中到 URDF 一处；`chassis_bringup` 与 nav 栈共享同一 description。
- **负面**：
  - **发布者二义性代价**：nav 栈与 `chassis_bringup` 都含 `robot_state_publisher`，两者同跑会
    双发 `base_footprint→base_link`；`slam.launch.py` 增 `publish_robot_tf` 开关来消歧——
    即多了一个必须记住的运行约束。
  - `base_link` 语义变更（footprint 中心 → 轮系中心）波及所有硬编码 `base_link` 的配置
    （nav2/EKF/collision_monitor），需上车复测；旧注释/旧 bag 需显式迁移。
  - `world` 引入后，机械臂 demo 的 `world→base_link`（`rak_moveit_config`）与移动底盘
    `world→map→…` 不能同场合并——沿用既有约束，进一步固化为"两套 world 不同场"。
  - z 高度与前后距离仍是占位值：地图质量对 `lidar_z` 误差敏感（旧流程已注明"小而静默的永久误差"）。

## 被否决的方案

- **保留旧约定"`base_link` 即 footprint 中心、无 `base_footprint`"**：实现最少改动，但把导航
  参考点绑死到车体中心，与 REP-120 相悖，且 `rak_chassis.urdf.xacro` 里已存在的 `base_footprint`
  变成僵尸帧——两个来源长期打架。
- **传感器静态 TF 继续放 launch 的 `static_transform_publisher`**：分散几何来源、与 RSP 双发布，
  且每加一个传感器都要改 launch；违背"每帧唯一发布者"。
- **`map` 直接作为全局根（暂不引入 `world`）**：省一层，但 `map` 随 SLAM 会话重建/重定位跳变，
  不是无界锚点；无限开放世界迟早要拆，晚拆代价更大。
- **在 URDF 里定义 `camera_depth_optical_frame` 等光学帧**：外参未标定且与 Orbbec 驱动自发布
  冲突（ADR-0014），会造出两个光学帧发布者。
- **机械臂安装点放 `rak_wbc` 的 `wbc_base.urdf.xacro`（保持现状）**：那是 Pinocchio 分析模型，
  不发布 TF；机器人 description 需要自己的 `arm_base_link`。两处值保持同步（见待闭合项）。

## 待闭合项

1. **`TODO(measured)` 回填**：`base_height`（车体离地）、`lidar_z`（雷达高）、`mic_z`、`cam_x/z`、
   `imu_x/z`、`arm_x/z`（臂安装面高）——装车标定后填回 `rak_chassis.urdf.xacro`。
2. **`world → map` 全局锚点未实现**：现为静态恒等占位；户外 ECEF/UTM 或多地图注册模块待设计。
3. **相机外参未标定**：`base_link→camera_link` 按 ADR-0014 实测；深度/彩色光学帧由驱动发布。
4. ~~**机械臂六轴链未接入 RSP**~~ ✅ **已闭合（2026-10-15）**：`rak_chassis.urdf.xacro`
   里的臂从"零长度控制桩链"换成了**真实运动链**——几何单点定义在
   `src/rak_description/urdf/macros/arm_chain.xacro`，由本文件与
   `rak_arm.urdf.xacro` **共用同一个宏**（此前是两份互不相干的东西：一份真实几何、
   一份空壳）。RSP 现在会发 `arm_base_link→shoulder_link→…→tool0`。
   本机实测 `tf2` 查 `arm_base_link→tool0` = **(0.10, 0, 0.33)**（Home 位）✅。
   `RakSystemHardware::on_init` 的 11 关节/顺序/接口契约不变，由
   `src/rak_chassis/test/validate_chassis_urdf.py` 钉住（已入 `colcon test`）。
5. **`wbc_base.urdf.xacro` 的 `arm_mount_joint` 为镜像**：值与 `rak_chassis.urdf.xacro` 的
   `arm_mount_joint` 必须同步（ADR-0011 待闭合 #2/#3）。
6. **REP-120 迁移需上车复测**：`odom→base_footprint` 改动后，EKF 收敛、rf2o 里程、Nav2 跟踪
   与 `collision_monitor` 需在实机重新验证；旧 rosbag 的 `odom→base_link` 需显式迁移。
