# 底盘 + 视觉寻路 v1.0 设计 —— 纯视觉可行驶区域 + 固定地图定位 + 全向 Nav2

> 状态：草案 · 最后更新：2026-11-15

- **日期**: 2026-11-15 · **状态**: 草案（待评审，接受后落 ADR）
- **目标**: v1.0 纯视觉寻路——前置相机可行驶区域分割 + 固定地图无漂移定位 + MPPI 全向控制。
  4D 寻路（深度相机/多相机/运行时雷达）是 v2.0，本文不展开。
- **关联**: `docs/design/chassis_control.md`（死区补偿的固件侧依据）· ADR-0007（ros2_control）·
  ADR-0016（CAN 唯一传输）· ADR-0022/0024（CANopen v5 主机侧）· ADR-0023（时间基准）·
  `src/rak_road_perception`（现有分割节点）· `src/rak_bringup/config/nav2_params.yaml`

---

## 0. 实测基线（2026-11-14/15，本机，全部可复现）

### 0.1 算力包络（trtexec 10.16，10 W 模式，FP16，batch1，useCudaGraph）

| 网络 | 输入 | 参数 | FLOPs | GPU 均值 | 吞吐 |
|---|---|---|---|---|---|
| 合成 conv 基准 small | 360×640 | 0.29M | 3.96G | 4.94 ms | 202 qps |
| 合成 conv 基准 mid | 360×640 | 1.16M | 13.5G | 8.08 ms | 124 qps |
| 合成 conv 基准 big | 360×640 | 2.03M | 25.5G | 13.44 ms | 74 qps |
| TwinLiteNet8 真模型（果园 7 类，作架构参照） | 360×640 | 0.44M | — | 20.4 ms | 48.5 qps |
| **TwinLiteNet+ Small（真权重，DA 头）** | **384×672** | **0.12M** | — | **16.87 ms** | **59.1 qps** |
| **TwinLiteNet+ Medium（真权重，DA 头）** | **384×672** | **0.48M** | — | **33.28 ms** | **29.9 qps** |
| TwinLiteNet8 端到端（trtexec 含 H2D/D2H） | 360×640 | | | 21.5 ms | 46.5 FPS |

结论一：**这块板 10 W 模式跑 384×672 FP16 分割，纯算力预算 ≈ 每帧 30 ms**。
Small 16.9 ms（+10 ms CPU 前后处理 = 27 ms/37 FPS）达标；Medium 33.3 ms
（≈43 ms/23 FPS）是边界可用。选型看**实测**，不看 FLOPs——TwinLiteNet8
只有 1/8 的 FLOPs 却比 Small 慢（PRelu/Slice/MatMul 结构效率低）。

结论二：**端到端预算 ≈ GPU + 10 ms**（Python 侧 numpy/CPU 预处理 4.1 ms + 后处理 6.3 ms 实测）。
32 ms/帧（31 FPS）可行；48 ms（20 FPS）是保守线。

**端到端实测（`scripts/compare_roadseg.py`，1280×720 输入，含 resize 与掩码回放大）**：
TwinLiteNet+ Small **44.5 ms/帧（22.5 FPS）**。分解：TRT 单次调用 25.4 ms
（execute 15.9 + sync 2.2 + 拷贝 1.5 + 后处理 ~5.9），其余是一次 1080p→672×384
下采样和一次 672×384→1080p 概率图上采样。**上采样比推理还贵**，所以 v1.0 的
消费者若不需要全分辨率，应直接用模型分辨率的掩码。

结论三：**TensorRT 引擎构建很贵**（Small 377 s、Medium 447 s、TwinLiteNet8 367 s，
合成网 60–150 s）——引擎必须本机缓存，绝不能在每次上电时构建。任何"启动时动态优化"
的设计在这块板上是错的。

**读数的适用条件**：以上全部在 `nvpmodel` 10 W 模式、**无桌面会话**（本机当前无
gdm/GNOME）、`jetson-clocks.service` 未运行、CPU governor `schedutil` 浮动的前提下测得。
换功耗模式、开桌面会话或锁频后数字会变——**不要跨状态比较**（对齐
`docs/reference/machine_baseline.md` 的既有告诫）。

### 0.1b A/B 实测：LFD-RoadSeg（现役） vs TwinLiteNet+（候选）

同一批图、同一台机、`scripts/compare_roadseg.py`（6 张 1280×720 BDD100K 街景，
含夜间与白天；LFD v6 是室内权重，域不匹配 —— 这里比的是**延迟与边界锐度**，
不是该场地的精度结论）：

| 模型 | 输入 | 精度 | 单帧（含 resize/回放大） | p50 | FPS | soft/bnd↓ |
|---|---|---|---|---|---|---|
| **TwinLiteNet+ Small** | 672×384 TRT FP16 | fp16 | **40.3 ms** | 40.1 ms | **24.8** | **2.05** |
| LFD-RoadSeg v6 | 640×480 TorchScript CUDA | fp16 | 190.8 ms | **66.5 ms** | 5.2 | 12.51 |

- 首帧含 JIT 编译：TwinLiteNet+ 154 ms、LFD 1185 ms（都只发生一次）。
- `soft/bnd` = 概率落在 (0.25, 0.75) 的像素数 ÷ 边界像素数，**越小边界越锐**。
  TwinLiteNet+ **锐 6 倍**，分辨率还更高。
- 目视（`/tmp/roadseg_ab/*_ab.png`）：TwinLiteNet+ 掩码贴合车道；LFD v6 在
  街景里把车辆/引擎盖/路缘一起涂成"可行驶" —— 室内权重的域偏移，符合预期。
- 结论：**延迟与边界两项 TwinLiteNet+ 完胜，可以直接替换**；精度结论要等
  本场地实拍数据（TODO P1「50 帧边界评估」）。

### 0.2 已实测的模型导出链（本机可复现）

TwinLiteNet+ 官方无 ONNX 导出脚本，本机已跑通并实测：

1. 权重：官方 Google Drive（`gdown --folder`），nano/small/medium/large 四份 .pth，
   `load_state_dict(strict=True)` 零缺失零多余（实测）。
2. **输入必须 384×672，不能 360×640**：CAAM 的 `patch_split` 要特征图能被
   `(2,4)` bins 整除，360×640 在 forward 里直接 `RuntimeError`。这是官方
   README 未写的硬约束，写进本文档以免下次再踩。
3. 导出脚本已入仓：`scripts/export_twinlite_onnx.py`（本机实测跑通，
   0.122M 参数，输出双头 `da_logits` + `lane_logits`）。脚本自带两条前置检查
   （numpy<2、matplotlib stub）与导出后 shape 断言。
4. `trtexec --fp16 --useCudaGraph` 构引擎：Small 377 s、Medium 447 s
   （引擎必须缓存，白名单进 `data/models/`）。
5. 导出环境的坑：仓库 `model.py` 顶层 `import matplotlib.pyplot` 与 `import cv2`，
   而 export 需要 numpy 1.x 环境（系统 cv2 是 numpy1.26 编的）→ 需 stub
   matplotlib + numpy<2。这条只影响开发机导出流程。

### 0.3 环境事实（决定软件选型）

| 事实 | 影响 |
|---|---|
| torch 2.12.0+cu132 **不在**系统 Python（pip index 有，属用户 site 域，需用户确认安装域） | 训练/导出在开发机（RTX 3080/5090）做，板上只跑 TensorRT |
| `tensorrt` 10.16.2.10 Python 绑定**存在**，可加载 .plan | 板上推理可用 TRT Python API，无需 onnxruntime |
| onnxruntime-gpu 1.32 wheel 可下；CUDA EP 实测 54.9 ms（慢于 trtexec 21 ms） | **ORT CUDA EP 不是部署路径**；TRT EP 首跑仍走引擎构建 |
| trtexec 在 /usr/bin | 引擎构建/验收工具链齐全 |
| cv2 4.6.0、numpy 1.26.4、Jetson 直连 IMU（SPI）驱动已就绪 | 预处理在 CPU 做（<10 ms），先不引入 CUDA 预处理复杂度 |
| 磁盘 185G 空闲，可放训练数据/模型缓存 | 固定场地标注数据可落本机 |

### 0.4 本机 Nav2 / MPPI 现状（关键约束）

- `ros-jazzy-nav2-mppi-controller` = **1.3.13**（apt 当前唯一候选）。critics.xml 实测只有
  12 个 critic —— **没有 `OmniConstraintCritic`、没有 `MecanumCritic`**（1.3.14 才引入）。
  `motion_model: Omni` 与 vy 参数在 1.3.13 可用（lib 实测确认 OmniMotionModel 存在）。
- 社区证据（PR #6460，未合并）：麦轮 45° 斜走只驱动两个轮子，实车会滑 —— v1.0 要
  **避开速度空间对角线**。1.3.13 上的等价手段：`TwirlingCritic` + 小 `vy_std`/`wz_std`
  + `ConstraintCritic` 收紧 vy 包络，而不是等 1.3.14。
- `wz_max=0` 不可行（源码 clip 逻辑 + GoalAngleCritic 矛盾，见 control 调研）。
- RPP 控制器不写 `linear.y`（源码确认）——用了等于放弃麦轮，排除。
- `nav2.launch.py` 目前硬性包含 lidar+slam 链；v1.0 需要一个 `sensors:=camera` 的旁路
  （见 §4）。

---

## 1. 架构（v1.0 数据流）

```
                    ┌──────────────────────────────────────────────┐
 [前置相机 30fps] ──▶│ rak_road_perception（升级：TRT FP16 分割）     │──▶ /road_perception/road_mask
                    └──────────────────────────────────────────────┘         │
                    ┌──────────────────────────────────────────────┐         │
 [Jetson SPI IMU]──▶│ rak_imu_driver（已有）                        │──▶ /imu/data_raw
 [麦轮里程计]──────▶│ mecanum_drive_controller（ros2_control 已有） │──▶ ~/odometry
                    └──────────────────────────────────────────────┘         ▼
                    ┌──────────────────────────────────────────────┐   ┌──────────────┐
 [离线雷达建图地图]──▶│ map_server（存 once，v1.0 不跑 slam_toolbox） │   │  Nav2 全栈    │
 [AprilTag 锚点]───▶│ rtabmap localization（纯定位模式，可选）       │──▶│  (MPPI Omni) │
                    └──────────────────────────────────────────────┘   └──────┬───────┘
                                                                              ▼ /cmd_vel
                                                                    velocity_smoother → collision_monitor
                                                                              ▼
                                                              mecanum_drive_controller → CANopen v5 → MCU
```

分层职责（对齐用户"极其完美"清单）：

| 层 | v1.0 实现 | 不做（留给 v2.0） |
|---|---|---|
| 感知 | TwinLiteNet 系 360×640 FP16 TRT + 边界后处理 | 深度相机、多相机融合 |
| 定位 | rtabmap 纯定位（双目/单目）+ AprilTag 锚点 + EKF | 4D 场景一致性 |
| 全局规划 | NavfnPlanner / SmacPlanner2D（固定地图，静态障碍） | 动态重规划 |
| 局部控制 | MPPI Omni（1.3.13 critic 集，§5 参数） | MecanumCritic（1.3.14） |
| 执行 | 死区补偿进 MCU 固件（对齐 chassis_control.md）+ 在线尺度标定（P1） | 4D 地形代价 |

---

## 2. 感知层：分割模型选型

### 2.1 结论

- **主选：TwinLiteNet+ Small（BDD100K 预训练 DA 权重，MIT license）**，
  **384×672** 输入，TRT FP16。本机实测 **16.87 ms / 59 qps**（10 W 模式，
  含 H2D/D2H 端到端 ≈18 ms）。加 CPU 前后处理 10 ms → 27 ms ≈ 37 FPS。
  权重可从官方 Google Drive 下载（本机已缓存四份 .pth），导出链见 §0.2。
- **备选：TwinLiteNet+ Medium**（0.48M / mIoU 92.0）——本机实测
  **33.28 ms / 30 qps**，加前后处理 ≈43 ms ≈ 23 FPS。仅当 Small 边界明显
  不够细且能接受 23 FPS 时升级；这是"精度换帧率"的明确取舍点。
- **排除 GMSANet**（仓库只有 "Waitting upload"，无代码无权重）、
  **排除 AURASeg**（Jetson Nano 实测 3.12 FPS）、**排除 TwinLiteNet+ Large**
  （1.94M，按 Small→Medium 的 1.97× 推算 ≈65 ms，10 W 模式不达标）。
- **TwinLiteNet8（HF WEN0256）只作架构参照**：果园 7 类域不匹配，但它的
  `export_onnx.py`/`predict_onnx.py`/`JETSON_DEPLOY.md` 是现成的部署模板
  （本机已下 `twinlite8.onnx` 并实测 TRT FP16 20.4 ms / 48.5 qps）。

### 2.2 分辨率与预处理（实测依据）

- 输入 **384×672**（非 360×640，CAAM 硬约束，见 §0.2）。不改方图 224——
  改了精度和边界都受损。
- 双输出只取 `da_logits`（可行驶区域）；`lane_logits` v1.0 不用（固定地图不需要
  车道线），保留输出是为了和官方权重口径一致，若后续要压低延迟可在导出时删头。
- **归一化统计量换成 BDD100K 口径**（TwinLite 官方 train.py 的 mean/std），
  不沿用 LFD-RoadSeg 的 _MEAN/_STD（roadseg_node.py:259 现值是旧模型域）。
- 上采样顺序：**logits 先 bilinear 到原图，再 argmax/threshold**（不是 argmax 后
  最近邻放大）——这是"边界锐利"最便宜的一步，模型不动。
- 可选 guided filter / 形态学闭运算，等首帧实车掩码看了再决定（不预先加）。

### 2.3 部署路径（按实测排序）

1. **TensorRT Python API + .plan 文件**（首选）：板上 `tensorrt` 绑定已存在，
   `trt.hpp`（rak_gpu）已有 C++ 路径；.plan 由 `trtexec --fp16 --saveEngine`
   构建（一次性，367 s），构建产物进 `data/models/roadseg/`（gitignored）+
   配 SHA-256 记录（对齐 laya 的 engine 管理惯例，`docs/how-to/laya_tensorrt_runbook.md`）。
2. onnxruntime-gpu CUDA EP 只作**校验器**（导出后数值对拍用），不是部署路径
   （54.9 ms 太慢）。
3. **不做 INT8**：PTQ 掉 2.1 pp mIoU（TwinLiteNet+ 论文 Table 5），QAT 成本高；
   FP16 已 20 ms，余量足够。v2.0 若需功耗优化再启用。

### 2.4 节点改造（`rak_road_perception`）

- `roadseg_node.py` 换推理后端：torch.from_numpy → TRT `IExecutionContext.execute_v3`
  （输入 1×3×384×672 FP16 pinned buffer，输出 da 头 1×2×384×672 FP32）。
- 保留 shared_memory feeder / ROS image 两条输入路径与 `road_threshold` 参数
  （语义不变：road 类概率阈值）。
- 新增参数 `engine_path`（默认 `data/models/roadseg/twinlite_plus_small_fp16_384x672.plan`）、
  `model_input_wh: [672,384]`、`engine_output_name`（多输出引擎选头，空=第一个）、
  `normalize`（`unit` = TwinLiteNet+；`imagenet` = LFD-RoadSeg，A/B 时用）。
- 二分类头用**一个 sigmoid** 代替 2 路 softmax：源码实测省 ~5.5 ms/帧
  （30.9 → 25.4 ms），占单帧预算的四分之一。
- `_require_torch()` 路径删除（板上无 torch 是常态，不是故障）。
- 保留 `/road_perception/road_mask`（mono8）与 overlay 话题，**不改话题名**——
  下游 Nav2 观察源已按此接线。

---

## 3. 定位层：固定地图 + 视觉重定位（锁死漂移）

### 3.1 结论

**主路径：RTAB-Map 纯定位模式（apt 0.23.13 二进制）+ AprilTag 锚点 +
robot_localization 双层 EKF。** 纯 VO 兜底，ORB-SLAM3 不做（GPLv3 +
Pangolin 钉版本 + 4GB 板上无收益，调研已证）。

关键事实（调研核实）：
- `ros-jazzy-rtabmap-ros` 0.23.13 在官方 apt（本机 apt-cache 实测），CPU-only，
  68 MiB，无 CUDA 依赖 → 4GB 板可行。
- 纯定位参数：`Mem/IncrementalMemory=false` + `Mem/InitWMWithAllNodes=true`
  （rtabmap_launch 的 `localization:=true` 即此配置）；`RGBD/StartAtOrigin=true`
  或发 `/initialpose`。
- rtabmap 直接吃 apriltag_ros 的 detections 当 landmark（`Marker/Priors`，
  需 `Optimizer/PriorsIgnored=false`——**默认 true 会静默忽略**，这是坑）。
- AprilTag 布设成本极低（打印 5–10 个 tag 贴固定场地），换来的绝对位姿锚定
  是"固定场地"场景下性价比最高的选择。

### 3.1b AprilTag 链路实测（无相机，合成图，2026-11-15）

`apriltag_ros` 3.4.0（apt）已在板上装好，用合成图跑通整条链（工具逻辑同
`scripts/compare_roadseg.py` 的思路：不需要硬件也能验接线）：

- 输入：1280×720 mono8 合成帧，`camera_info` fx=fy=800，36h11 tag 边长 **0.16 m**
  渲染成 **64 px** → 理论距离 `z = fx·size/px = 2.00 m`。
- 结果：**45 帧检测到 44 次（97.8%）**，`id=0` 正确，中心坐标中位数
  **(632.5, 232.5) px**，理论 (632.0, 232.0) —— 亚像素级。
- 资源：`apriltag_node` 在 30 Hz、1280×720 下 **≈6% 单核 CPU、RSS 100 MB**。
- 话题：只发 `/apriltag/detections`（`apriltag_msgs/AprilTagDetectionArray`，
  含 corners + homography，**不带位姿**）。位姿由此计算：rtabmap 的 `tag_topic`
  参数直接吃这个数组自己做 PnP —— 这就是为什么 §3.1 强调
  `Optimizer/PriorsIgnored=false`，否则检测到了也不会进地图。

标定入口：`ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025 image:=/camera/image_raw camera:=/camera`，
存到 `~/.ros/camera_info/ax__usb2.0:_ax_usb2.0.yaml`（v4l2_camera 的默认路径），
或显式传 `camera_info_url:=file:///home/xrak/rak/data/calib/front_camera.yaml`。

未测（需相机/需要建图）：真实光照与运动模糊下的检出率、`Decimate` 对远处小 tag 的影响、
rtabmap 纯定位在 4GB 板上的 CPU 占用（测法：`sensors:=camera` 起栈后看
`/rak/metrics` 与 `top -H -p $(pgrep rtabmap)`，1 Hz 检测率下预期单核 30–60%）。

### 3.1c 标签布设设计（打印表）

| 项 | 值 | 理由 |
|---|---|---|
| 家族 | `36h11` | apt 默认支持，误检率低 |
| 边长 | **0.16 m** | 2 m 处约 64 px；0.10 m 在 3 m 外只剩几十像素，PnP 抖动 |
| 材质 | 哑光纸/亚克力，无覆膜反光 | 反光会让 quad 检测失败（`Decimate` 也救不了） |
| 位置 | 每个拐角/路口/直道两端各 1 张，**中心高 0.5–0.8 m**，正对来车方向 | 单目俯视 FOV 下沿 0.4 m、上沿 4 m，标签要落在视场里 |
| 数量 | ≥ 6（一条 14 m 多拐角路线） | 任意时刻至少 1 张在视场内，2 张可解尺度 |
| 记录 | `tag_id -> (x, y, yaw)` 写进 `data/maps/*_tags.yaml`（gitignored） | 冲突时以实测 alias 表为准 |

### 3.1d 启动接线与所有权

`ros2 launch rak_bringup localization.launch.py database_path:=~/.ros/rtabmap.db`

| 组件 | 发布 | 谁拥有 |
|---|---|---|
| `rtabmap`（纯定位，`camera_model:=mono`） | `map->odom` TF、`/rtabmap/info` | 本 launch |
| `rob: EKF`（`ekf.yaml`） | `odom->base_footprint` | `slam.launch.py` / `sensors:=lidar` 档 |
| `apriltag`（命名空间 `apriltag`） | `/apriltag/detections` | 本 launch，可 `use_apriltag:=false` 关 |
| `map_server` | `/map` | `nav2.launch.py sensors:=camera map_yaml:=...` |

**三条硬约束**（写进 launch 注释了）：① 不得与 `slam.launch.py` 同跑（两个
`map->odom` 发布者）；② rtabmap 只发布 `map->odom`，`odom->base_footprint`
仍归 EKF；③ **相机内参必须先标**——不是双目标定，是单目 `fx/fy/cx/cy`。

### 3.1f 单目内参 vs 双目标定：v1.0 只需要前者

v1.0 的前视相机是**单目**，`camera_model:=mono` 是 launch 默认值。双目标定
（内参 + 校正 + 基线外参）**不在 v1.0 范围内**；那双目镜头留给机械臂末端的
eye-in-hand（近距、需要外参到手眼链），不要占用前视位置。

但**单目内参必须标**，理由是硬的：

| 消费者 | 需要什么 | 没内参会怎样 |
|---|---|---|
| `apriltag` PnP | `fx, fy, cx, cy`（畸变可选） | 只能给像素坐标，给不出米制位姿；标签锚点失效 |
| `rtabmap` mono | 同上 + 尺度来自 EKF odom | 特征三角化退化，重定位不出位姿 |
| IPM 地面投影（S3） | 同上 + 相机高度/俯仰 | 边界像素映射不到地面，比例全错 |

标定成本：棋盘格 + `camera_calibration`（apt 自带）约 **10 分钟**，产出
`file://.../camera_calibration.yaml` 填进相机 launch 的 `camera_info_url`。
`camera_model:=stereo` 保留但只在"底座真的装了已标定双目"时才用。

尺度从哪来（单目的老问题）：**EKF 的轮速+IMU 里程计提供尺度先验**，
AprilTag 提供绝对锚。所以单目在"固定地图 + 有里程计"的条件下不抖尺度；
这也是为什么 §3.1 把它排第一而把纯 VO 放兜底。

### 3.1e 失败模式与处置

| 现象 | 原因 | 处置 |
|---|---|---|
| `/rtabmap/info` 里 `loopClosureId=0` 长期不变 | 数据库与当前场地不匹配，或 `Mem/IncrementalMemory` 没生效 | 核参数回显（`ros2 param get /rtabmap Mem/IncrementalMemory`），确认是 `false` |
| 标签检测到但地图不认 | `Optimizer/PriorsIgnored` 是默认 `true` | 用 `rtabmap_localization.yaml`（已显式 `false`） |
| `map->odom` 抖动 ±10 cm | 视觉特征不足（白墙/纯色地面） | 增加标签覆盖；不要把 Kp/MaxFeatures 调太低 |
| 位姿整体平移 | 数据库建图时的原点与实际不符 | 用 `/initialpose` 或 `RGBD/StartAtOrigin` 重新锚定 |
| rtabmap 一直 `Localizing...` 不收敛 | 相机内参没喂（`camera_info` 空） | 先做单目标定并发布 `camera_info`，再谈重定位 |

### 3.2 建图（一次性，离线）

- 用仓库现有 N10_P 雷达 + `slam.launch.py`（现有链路）建一次图存 `map.pgm/yaml`，
  rtabmap 数据库同场录制（`rtabmap-slam` 的 database.mk 备份）。
- 之后**运行时不跑 slam_toolbox**，改 `map_server` + rtabmap localization 模式。
  这消除了"slam_toolbox 与定位抢 map→odom"的冲突（现有 nav2_params.yaml 注释
  已预告这个切换路径）。

### 3.3 TF 树（v1.0 最终形态，ADR-0002 两层结构不变）

```
map ──(rtabmap localization, 纯定位)── odom ──(robot_localization EKF)── base_footprint ──(URDF)── base_link
```

- EKF `odom0`：mecanum_drive_controller 的 `~/odometry`（vx/vy 可用，yaw/vyaw
  按团队规矩排除——麦轮打滑）。**F1 真机验收后才接**（当前 P0 状态）。
- EKF `imu0`：`/imu/data_raw` 只用 vyaw（现配置已如此）。
- rf2o 退役为 v1.0 非默认（无雷达运行时）。
- **协方差仍全占位**——P1 的 Allan/直行回归标定是 v1.0 上线前置项，不是可选项。

---

## 4. Nav2 侧改造

### 4.1 launch 与话题

- `nav2.launch.py` 加 `sensors:=camera|lidar` 开关（默认 lidar 保持现状）：
  `camera` 时不 include slam.launch.py，改起 `map_server` + （可选）rtabmap。
  这消除了运行时雷达依赖，符合 v1.0 定义。
- collision_monitor 观察源：v1.0 保留 `scan_source` 配置但 `enabled: false`
  （无雷达运行时）；分割掩码的"负障碍"接入路径：
  `road_mask → (相机内参) → base_footprint 平面投影 → Nav2 costmap 观察源`。
  这条 v1.0 **只做减速**（进 PolygonSlow 语义），stop 级动作留给 v2.0 深度。
- `waypoints.yaml`（`rak_task_executor`）填入路口/拐角 alias（固定地图手工标定），
  路口处理 = waypoint alias + FollowWaypoints，不改 Nav2。

### 4.2 MPPI 参数（1.3.13 可用集，起点值）

```yaml
FollowPath:
  plugin: "nav2_mppi_controller::MPPIController"
  motion_model: "Omni"          # 1.3.13 已支持（lib 实测）
  vx_std: 0.15
  vy_std: 0.08                  # 压横移采样 → 避开 45° 斜走（PR #6460 实证）
  wz_std: 0.12
  vx_max: 0.25
  vx_min: 0.02                  # 沿用现有 stick-slip 钳位依据（chassis_control.md §6）
  vy_max: 0.15
  vy_min: -0.15
  wz_max: 0.50                  # 不要 0（clip 后终点姿态永远达不到）
  batch_size: 1000
  time_steps: 48
  model_dt: 0.05
  temperature: 0.3
  gamma: 0.015
  critics: ["ConstraintCritic", "CostCritic", "GoalCritic", "GoalAngleCritic",
    "PathAlignCritic", "PathFollowCritic", "PathAngleCritic", "TwirlingCritic"]
  TwirlingCritic: {enabled: true, cost_power: 1, cost_weight: 10.0}   # 压纯旋转
  # 其余 critic 沿用现 nav2_params.yaml 权重（PathAlign 10.0 等）
```

- 现配置已是 MPPI Omni + 这套 critic（nav2_params.yaml:51-132），**改动最小化**：
  加 TwirlingCritic、把 vy_std/wz_std 调小、其余不动，直道先实测再说。
- 升级 1.3.14（自编译 nav2-mppi-controller 包）是 P2：拿到
  `OmniConstraintCritic`/`MecanumCritic` 后替换手工约束。

---

## 5. 执行层：死区补偿与在线标定

- **比例死区补偿进 MCU 固件**（对齐 `chassis_control.md` 已有结论：死区是
  PWM→轮速非线性，闭环在 TIM 25 kHz 位置环内，跨 CAN 补偿是开环前馈，放主机错）。
  麦轮地毯死区 PWM 30–35 的量级参考（Zenodo preprint 2026-05，可信度：单作者
  单平台，只当量级不当定律）。
- **主机侧只做**：
  - MPPI `VelocityDeadbandCritic`（1.3.13 已有，critics.xml 实测确认）——
    `deadband_velocities` 等实测死区对应的车体速度后填入（P1）。
  - 麦轮里程计**在线尺度标定**（Sensors 26(18):5692 两级闭环方法）：用
    rtabmap 绝对位姿做真值，标 `wheels_radius`/`sum` 尺度因子，P1 实施。
- **多阶段 Goal Manager**：v1.0 不引入（论文实现是独立 ROS 节点，侵入决策层）。
  路口确定性用 waypoints alias + 全局路径分段保证。实车若出现拐角卡滞，
  再评估单点 kickstart（IMU 判堵转）作为 v1.1。

---

## 5A. 相机几何：单目 / 双目 / 深度，v1.0 怎么选

### 5A.1 三种几何分别买到什么

| 方案 | 买到的能力 | 代价 |
|---|---|---|
| **单目（现状）** | 分割掩码；地面投影靠已知相机高度+俯仰的 IPM（单应），平坦地面成立 | 无度量；俯仰/高度标定误差按 `r·Δθ` 线性放大；坡道/门槛直接失效 |
| 双目（侧肩 640×720 单眼） | 度量深度；stereo VO 有尺度，可替代 AprilTag 做绝对位姿 | 完整内外参标定 + 校正；弱纹理地面在掠射角出现视差空洞；小基线误差 ∝ z² |
| 深度相机（Astra / Gemini 335L） | 近场稠密度量、抗弱纹理（主动光） | 主动传感器=IR 发射；FOV 窄；算力/带宽/内存；属 v2.0 定义 |

### 5A.2 实测/算式预算（决定取舍的数字）

**双目深度误差** `Δz = z²·Δd/(f·B)`，取 f≈457 px（640 宽单眼、~70° HFOV）、
B≈65 mm（USB SBS 模块典型基线）、亚像素 Δd=0.25 px：

| 距离 | 1 m | 2 m | 3 m | 5 m |
|---|---|---|---|---|
| Δz | 0.8 cm | **3.4 cm** | 7.6 cm | 21 cm |

**单目 IPM 俯仰敏感度**，地面点误差 ≈ `r·Δθ`：

| 距离 | 1 m | 2 m | 3 m |
|---|---|---|---|
| Δθ=0.5° | 0.9 cm | 1.7 cm | 2.6 cm |
| Δθ=1.0° | 1.7 cm | 3.5 cm | 5.2 cm |

结论：**在 0.5–2 m 这段"寻路真正在意"的区间，单目 + 标定到 ±0.5° 与双目的
精度同级（1.7 cm vs 3.4 cm，单目还更准）**。双目唯一稳赢的是 3 m 以外和
坡道处，而固定场地 + AprilTag 已经解决"米级以上"的定位。

### 5A.3 为什么 v1.0 不加双目

1. **它解决的问题 AprilTag 已经解决**：双目给的是"有尺度的绝对位姿"，
   固定场地一角贴 1 个 tag 就给到厘米级且无漂移，成本 0 行代码。
2. **它要付的账很重**：`rak_stereo_driver/src/capture_node.cpp:127` 只构造了
   `CameraInfoManager`、**从未发布 camera_info**，且仓内无任何标定文件
   （`stereo.launch.py:19` 两个 `camera_info_url` 默认空）——双目路径的真实
   前置是完整标定 + 校正，半天起步。
3. **它在关键区间的失败模式最坑**：室内平整地面在掠射角弱纹理，SGBM/GPU
   视差在"路边缘"恰恰出空洞，而路边缘精度正是这条链路的全部意义。
4. **它还要新代码**：640×720 双目匹配 CPU SGBM 约 100–200 ms，不达标；
   要么 VPI4 `StereoDisparity`（`rak_gpu` 已有 `VpiRectifier`，可接），
   要么 NVIDIA 立体模块——v1.0 不值得开这条支线。

### 5A.4 深度相机的正确角色

- **进 v1.1 做安全通道，不进 v1.0 做规划源**：Astra 0.6–8 m / Gemini 335L
  近场更密，抗弱纹理——正好补单目"看不见障碍高度"的缺口；
  接线方式 = `collision_monitor` 的观察源（减速/停止），不参与全局/局部规划。
- 用户提的"偏下 + 水平放置"是对的：水平朝前给 0.6–3 m 障碍带，与单目
  "偏上斜下看路面"互补。两台相机喂**不同消费者**时不需要互相标定
  （省一次外参标定）；将来要融合再上同一刚性支架。
- v1.0 硬件已接（见 §5A.6）；深度相机按 §5A.4 的「安全通道」定位，属 v1.1。
- **双目光学留给机械臂末端（eye-in-hand）**，前视位置只放单目：末端双目要的是
  近距离稠密视差 + 手眼外参，跟前视寻路是两套标定，混用会把两侧都做糊涂。

### 5A.5 安装几何建议（单目）

- **下倾 20–30°**，不是水平：掩码像素落在近处地面上，边界像素误差映射到
  地面的误差 ∝ 距离，下倾等于直接买精度。
- 视场下沿落在车前 0.4–0.5 m（轮子接地附近），上沿落在 4 m 左右。
- 一次性标定：内参（棋盘格）+ 相机高度 + 俯仰，目标 ±0.5°；标定方法可复用
  雷达建的那张地图的静态点做地面拟合（免买标定板）。
- 减振：麦轮辊子高频振动是真实模糊源（`vx_max=0.25 m/s`、30 fps 下帧间位移仅
  8 mm，运动模糊可忽略，振动不可忽略）——硅胶垫/泡棉垫即解决，0 行代码。

---

### 5A.6 前视相机实测（2026-11-15，硬件已接）

| 项 | 事实 | 来源 |
|---|---|---|
| 设备 | `/dev/video0`（`video1` 是 UVC metadata 节点，无格式） | `v4l2-ctl --list-devices` |
| USB ID | `05a3:9230` ARC International Camera（"AX USB2.0"） | `lsusb` |
| 形态 | **单目**：单帧铺满 1280×720，**不是 SBS 双目** | 抓帧目视 |
| MJPG | 1280×720@31、1920×1080@30 | `--list-formats-ext` |
| YUYV | 640×480@30；1280×720 只有 9 fps | 同上 |
| 安装 | 2026-11-16 已**物理装正**；此前为上下颠倒 | 抓帧目视 |
| 对焦 | **无对焦控件**（固定焦距）；`sharpness` 0–6 可调 | 同上 |

`v4l2_camera` 0.7.3 **不支持 MJPG**（`Current pixel format is not supported yet: MJPG`
→ cv_bridge `Unrecognized image encoding` 死）。720p@30 的可用路径是官方 `usb_cam`
（`mjpeg2rgb`），实测 **29.4 Hz、单核 50.5%、RSS 90 MB**。`driver` 开关两档都在
`camera.launch.py`；默认 `usb_cam`，理由见 §5A.7（分辨率决定标定可用性）。

### 5A.7 单目标定结果（2026-11-15，板上实标）

标定板：ChArUco **5×7 方格**（OpenCV 索引口径；人眼数 7×5，整体 20×14 cm）、
方格 0.0286 m、marker 0.0214 m、`DICT_5X5_50`。

**板型是测出来的**：检测到的 marker 质心与两种板型的理论质心做单应拟合，
5×7 残差 **0.70 px**、7×5 残差 **97 px** —— 直接排除后者。

| 项 | 值 |
|---|---|
| 分辨率 | **1280×720**（必须等于运行分辨率，内参不缩放） |
| 视角数 | 24（每帧 10–12 角点） |
| in-sample rms | **0.401 px** |
| hold-out rms | **0.381 px**（12 标 / 12 验） |
| fx, fy | 844.40, 851.46 |
| cx, cy | 614.20, 401.18 |
| k1, k2, k3 | +0.2428, −1.4443, **0（固定）** |
| p1, p2 | +0.0011, −0.0001 |

**k3 固定 0 的原因**：自由拟合 |k3|=1.82 但 rms 完全不变（0.401 对 0.401）——
板没铺满整帧边缘，高阶项没有信息，k3 在吸误差。脚本自动判断并打印。

**720p 而不是 640×480**：同一块板在 640×480 下只够 6 个可用视角（欠约束），
720p 下 24 个。

**三个静默失败的坑（都实测过）**：

1. `cv2.FileStorage` 写的 yaml 被 ROS 拒（`invalid node; first invalid key: "rows"`）
   → 驱动**静默退回未标定**，只 warn 一句。改纯 yaml 文本写出。
2. `camera_name` 必须匹配驱动推导名（v4l2_camera 会 warn
   `[ax__usb2.0:_ax_usb2.0] does not match`，然后发全零内参）。`usb_cam` 无此限制。
3. `image_flip_node` **不转发 `camera_info`**：`use_camera_info:=true` 走精确时间
   同步，两路时间戳对不上就什么都不发；`false` 图像通但 info 依然没有。这也是当初
   翻转不做在 launch 的原因。**2026-11-16 相机已物理装正**，launch / `roadseg_node`
   / 标定脚本里的 180° 旋转逻辑已全部删除；物理旋转与软件旋转等价，标定 yaml 继续有效。

**模糊**：不是对焦——没有对焦控件。`sharpness` 扫描 0/2/4/6 = Laplacian
37/62/99/139，**已设 4**（6 开始掉 marker 检出）。

### 5A.8 多相机：DeepStream 9.2 / Holoscan 5.0（2026-11-15 实测）

已装 `deepstream-9.2` 9.2.0-1、`holoscan` 5.0.0-1（**仅 C++ 头文件，无 Python 绑定**）。

| 项 | 结果 |
|---|---|
| DeepStream 单相机 + nvinfer 跑 TwinLiteNet+ | 29.9 FPS，GPU 53–98%，整机 8.9 W |
| batch=1 GPU 时间 | 19.34 ms（51.5 qps） |
| batch=6 GPU 时间 | 134.5 ms/批 = **44 帧/秒总量**，比串行只赚 15% |
| `nvv4l2decoder` 解 MJPG | 失败（`Internal data stream error`，io-mode 2/4 均试） |

batch 收益低是因为模型**访存受限**（§0.1：0.09 TFLOP/s）。六相机不能靠 batch，
必须降分辨率或帧率。先决条件：解决 MJPG 的 GPU 解码（`extra-controls` 或换 H.264
相机），否则 6×50% CPU 解码 ≈ 3 核是第一瓶颈。Holoscan 的价值在传感器直通、
不在 ROS 2 集成，v1.0 不引入。

### 5A.9 TTS 后端实测（2026-11-15）：TRT 不可用，kokoro 是唯一可行解

`sherpa-onnx` 2.0.1 pip wheel（aarch64）**不含 GPU 编译**（`GetSessionOptionsImpl`
明确要求 `-DSHERPA_ONNX_ENABLE_GPU=ON`）。官方 aarch64 GPU 包（v2.0.1，
`...-shared-gpu-onnxruntime-1.18.1`）内含 `libonnxruntime_providers_tensorrt.so`，
但 sherpa 自己的 provider 枚举**只接受 cpu/cuda**，传 `tensorrt`/`trt` 分别报
`Unsupported string: tensorrt` 与 `Tensorrt support for Online models only,Must be
extended for offline and others` —— **离线 TTS 走不了 TRT，这是上游未实现，不是配置问题。**

| 后端 | 合成（"前方道路畅通，可以出发。"，2.58 s 音频） | RTF |
|---|---|---|
| sherpa python / CPU，4 线程 | 5.74 s | 2.22 |
| sherpa C++ / CPU | 8.45 s | 3.27 |
| ORT 直接跑 kokoro ONNX，CPU EP | 0.487 s/推理 | — |
| ORT 直接跑 kokoro ONNX，CUDA EP | **0.378 s/推理**（快 22%） | — |
| ORT TRT EP | 0.678 s（更慢：TRT 把大图切碎，子图边界拷贝开销吃掉收益） | — |

**结论**：TTS 的瓶颈不在 EP。kokoro ONNX 单次推理只占整句时间的一小部分，
其余是 espeak 前端 + 声码器后处理 + Python 侧拼接（`len=1` 时 RTF 3.49，
`len=10` 时 RTF 1.84 —— 固定开销约 1.5 s/句）。**换 EP 救不了 TTS 延迟**；
要低延迟只能换模型（vits-aishell3 8 kHz 约 3× 实时）或走即时层预合成。

内存：kokoro int8（147 MB 下载 / 109 MB 模型）加载后 **RSS 408 MB**，
比 qwentts 的 1.2–1.5 GB 小 3–4 倍 —— **这才是换 TTS 的真正理由**。

**决定**：内容层默认 `backend:=sherpa`（kokoro int8），qwentts 降级为特殊情况手动
开。放弃为 TTS 找 TRT 加速：上游 offline TTS 没有 TRT 路径，且 ORT 实测显示
EP 不是瓶颈（CUDA 快 22% 但只作用在整句的一小部分时间上）。

### 5A.10 全栈高负载实测（2026-11-15，所有 TRT 组件同跑）

工具 `scripts/loadtest_full_stack.py`。同时运行：`usb_cam` 720p@30 + `roadseg_node`
（TensorRT）+ `apriltag_node` + `rtabmap` 纯定位 + Nav2 4 台服务器 + kokoro TTS
循环合成 + 两个静态 TF（替代尚不存在的 EKF 与 map→odom）。**Nav2 生命周期实测
达到 `Managed nodes are active`**，即所有服务器真的在跑，不是挂起。

| 组件 | 峰值 RSS |
|---|---|
| `roadseg_node`（TRT） | **614 MB** |
| TTS（kokoro int8，独立进程） | 465 MB |
| Nav2（4 服务器 + 2 lifecycle manager） | 348 MB |
| `rtabmap` 纯定位 | 249 MB |
| `apriltag_node` | 137 MB |
| `usb_cam` 720p | 128 MB |
| 静态 TF ×2 | 110 MB |
| **合计** | **2051 MB** |

整机：峰值 **2869 / 3520 MB**，最低可用 **650 MB**，swap 峰值 **756 MB**
（zram 压缩后实占约 200 MB）。TTS 循环 RTF 2.31–2.41 全程稳定，无推理失败。

**结论**：v1.0 全栈（不含 qwentts / Laya / 雷达 / 深度相机）**2.0 GB，可跑**，
留 650 MB 余量。加上 Laya TRT（~500 MB）+ 雷达（50 MB）后约 2.6 GB，仍在预算内。
qwentts（+1.2–1.5 GB）会把整机推到 4 GB 以上并大量换页 —— 这就是 §5A.9 决定
把内容层默认改成 kokoro 的直接依据。

**全模型版（含 Laya TRT + ASR + TTS，2026-11-15 第二次跑）**：

| 组件 | 峰值 RSS |
|---|---|
| **Laya TensorRT（BF16 引擎 807 MB）** | **1616 MB** |
| `roadseg_node`（TRT） | 353 MB |
| TTS（kokoro int8） | 289 MB |
| Nav2 | 204 MB |
| `usb_cam` | 132 MB |
| `apriltag_node` | 111 MB |
| `rtabmap` | 98 MB |
| ASR（zipformer 14M int8） | 93 MB |
| 静态 TF ×2 | 94 MB |
| **合计** | **2990 MB** |

整机峰值 **3455 / 3520 MB**，最低可用 **64 MB**，swap 峰值 **2049 MB**（几乎满）。
Nav2 仍达 `Managed nodes are active`，TTS RTF 3.06、ASR 67 ms/句、Laya 67.3 ms。
**这是压线：64 MB 可用内存意味着任何一次峰值都会触发 OOM kill。**
Laya 一个进程吃掉 1616 MB（引擎 807 MB + CUDA 上下文 + 工作区），是最大单体。
结论：**Laya 与全栈同时常驻不现实**，必须二选一或按需加载。

**实测暴露并修掉的一个真 bug**：`nav2.launch.py` 的 `sensors:=camera` 分支
原本不启 `robot_state_publisher`（那是 `slam.launch.py` 的事），导致相机档下
`base_link → odom` 永远拿不到，Nav2 生命周期在 `controller_server` 就
`Aborting bringup` —— **整栈看起来起来了，实际全部 inactive**。已修（相机分支
按 `publish_robot_tf` 自起 RSP），并在文档 §4.1 记录。

### 5A.12 语音交互延迟预算（实测，回答「能不能做到无感」）

全部数字来自本机实测，非估算。

| 环节 | 耗时 | 依据 |
|---|---|---|
| 唤醒（板端 R818） | <50 ms | 板固件判定，Jetson 侧只收 `/speech/wake` |
| ASR 流式解码 | **48–70 ms / 每秒音频** | 实测 379 次，zipformer 14M int8，101 MB |
| 端点检测（说完判定） | **0.5–1.2 s** | sherpa 默认 rule1/2/3（2.4/1.2/20 s），可调 |
| 决策（Laya TRT） | **67.3 ms** | 实测 20 次，BF16 引擎 |
| 即时层响应（预合成 wav） | **<100 ms** | 无推理，直接播文件 |
| 内容层响应（kokoro 合成） | **2.2–2.9 s**（短句） | 实测："好的" 2225 ms、"马上就来" 2648 ms |

**结论：能，但只有一条路能。**

**可行（≈1.7 s 首响）**：唤醒 → 流式 ASR → 端点检测 → Laya 67 ms → **即时层播预合成 wav**。
1.7 s 里 **1.2 s 是端点检测**，计算只占 0.17 s —— **瓶颈是"等你说完"，不是算力。**

**不可行**：内容层 kokoro 首字延迟 2.2 s 起（RTF 2.0–3.0），加上端点检测
就是 3.4 s，用户能明显感到"卡了一下"。所以 ADR-0018 的三层设计不是过度设计，
是**唯一能达标的结构**：内容层只负责"说完之后的那句话"，首响必须走即时层。

**真正做到"无感"要改的三件事**（都还没做）：

1. **不等端点，对部分结果反应**：`OnlineRecognizer` 每 50–70 ms 就有新文本，
   拿**部分假设**跑一个轻量意图分类（"起立"/"停"/"过来"），命中就立刻执行，
   不等句子结束。这把 1.2 s 的端点等待压到 ~0.3 s。
2. **即时层扩容**：现在只有预合成短句（`/speech/quick`）。要做到"一说话就起立"，
   动作类反馈应该**不走语音**——直接发动作，语音只做确认（"嗯"）。
3. **ASR 与决策并行**：现在 `asr_node` 出最终文本才发 `/speech/intent`；
   改成边解码边发中间意图，`rak_task_executor` 收到就预取路径（不等最终句）。

**风险**：部分结果误判会"抢动作"。缓解 = 高置信度才动作，低置信度只播"嗯？"
（即时层，<100 ms），等最终结果再纠正。

### 5A.11 内存预算（组件级估算，回答「全栈够不够」）

3520 MB 统一内存（GPU/CPU 共池），swap 2G zram + 2G file。swap 兜不住常驻
热数据（KV cache、TRT 权重），换出换入即可感知卡顿。

| 组件 | RSS | 依据 |
|---|---|---|
| `roadseg_node`（TRT + CUDA 上下文） | **625 MB** | 实测（引擎本身 1.6 MB） |
| `usb_cam` 720p | 90 MB | 实测 |
| `apriltag_node` | 100 MB | 实测 |
| `deepstream-app`（仅基准，不部署） | 605 MB | 实测 |
| qwentts TTS 内容层 | **1200–1500 MB** | AGENTS.md 实测 |
| ASR zipformer 14M | ~150–250 MB | sherpa-onnx 量级 |
| Laya TensorRT | ~400–600 MB | 引擎 + CUDA 上下文（同 roadseg 量级） |
| Nav2（已裁到 4 台） | ~300–400 MB | 上游经验值 |
| rtabmap 纯定位 | ~150–250 MB | CPU-only |
| N10_P 雷达 | ~50 MB | 串口驱动 |
| 深度相机驱动 | ~150–300 MB | SDK 运行时 |
| ROS 2 基础 | ~200–300 MB | DDS/daemon |

**全栈（qwentts 版）≈ 4.1–5.4 GB，超出 3.4 GB 物理内存——不够。**

**对策（按优先级）**：
1. **TTS 是唯一能砍的大头**（qwentts ≈ 全部缺口的来源）：
   `backend:=sherpa`（kokoro，量级小一个数量级）或按需加载（收到 `/speech/say`
   再拉起、说完退出，首帧 +1–2 s）。CosyVoice3（1.7 GB）已证明不可行，别再试。
2. Nav2 保持 4 台服务器（已裁，别回到全量 10 台）。
3. Laya 与 roadseg 各持一份 CUDA 上下文（~250 MB×2），合并单进程可省一份，
   v1.0 不值得。
4. 六相机 = 再挤 ~700 MB（6×v4l2 + CUDA 解码上下文）→ 必须配「按需 TTS」 +
   GPU 解码相机。

**v1.0 可行组合 ≈ 2.6 GB**（留 900 MB 余量）：usb_cam 90 + roadseg 625 +
apriltag 100 + rtabmap 200 + Nav2 350 + ASR 200 + Laya TRT 500 + 雷达 50 +
基础 300 + **sherpa TTS 150**。

---

## 5B. 板载 IMU 增稳 —— 设计（下位机代码已核对）

下位机事实源：`~/vehical_rak_motion`（底盘板）。本节结论基于读码，标注到文件行。

### 5B.1 先否掉"物理增稳"

增稳 = 陀螺 + **执行器**（云台）。底盘板只有陀螺，没有执行器，**不存在硬件级
物理增稳**。把"增稳"理解成"相机不抖"必须走机械（§5A.5 减振垫），IMU 一个 bit
也帮不上。

IMU 真正的增量在三件事上：**给地面投影提供实时姿态**、**给里程计做独立见证**、
**给时间基准提供采样时刻**。下面按可落地程度排序。

### 5B.2 现状（读码，2026-11-15）

| 事实 | 出处 |
|---|---|
| 一次 14 字节块读 = accel6 + temp2 + **gyro6**（Z 在 offset 12） | `Core/Src/app_imu.c:235-245` |
| **pitch/roll 只由加速度低通基线解算**（`s_acc_mg` + atan2），无陀螺参与 | `Core/Src/app_imu.c:285-287` |
| 倾斜结果经 `imu_tilt_deg()` 发布到 OD `0x2031`/`0x2032`，TPDO4 `0x481` @100 Hz | `Core/Src/app_canopen.c:205`、`docs/CANOPEN-V5.md:67` |
| 安装方向：模块竖装（+Y 朝上），符号约定**待现场核对**，核对前 Jetson 不消费 0x2031/0x2032 | `Core/Src/app_imu.c:275-290` |
| 冲击/饱和事件已存在（`IMU_EVT_IMPACT`/`SATURATED`），静止零偏重置已存在 | `Core/Src/app_imu.c:270-300` |

**关键缺陷（读码即可判定）**：纯加速度解算无法区分"倾斜"与"加速"。
车体沿 x 加速 a 时，重力向量在芯片系里多出一个 `a` 分量，pitch 直接读错
`atan(a/g)`：

| 加速度 | pitch 误差 | 2 m 处地面投影误差（`r·Δθ`） |
|---|---|---|
| 0.3 m/s²（起步） | 1.7° | **6.1 cm** |
| 0.5 m/s²（常用） | 2.9° | **10.2 cm** |
| 0.8 m/s²（MPPI `ax_max`） | 4.7° | **16.3 cm** |

也就是说：**加速时这路倾斜不但没用，还是错的**，而加速正是相机俯仰最大的时刻。

### 5B.3 设计 D1：互补滤波的 pitch/roll（下位机，约 25 行）

陀螺 X/Y **已经在同一个 14 字节块里**（`b[8..11]`），不需要新增总线事务、不需要
改 OD 契约：

```
每 10 ms（imu_update 的 100 Hz 节奏）：
  gyro_x, gyro_y  = ±500 dps 量程原始值 → °/s，减零偏
  pitch_acc = atan2 解算（现有公式，保留）
  pitch = α·(pitch + gyro_y·dt) + (1-α)·pitch_acc      # α=0.98 → τ≈0.5 s
  roll  = α·(roll  - gyro_x·dt) + (1-α)·roll_acc
  if |a| 偏离 1 g 超过 10%：信任陀螺支路（等同 (1-α)→0.05）
```

- 输出仍是 `0x2031`/`0x2032` 的 int16（量纲不变），**Jetson 侧零改动**。
- 零偏：现有启动零偏只标了陀螺 Z；D1 需要顺带标 X/Y（同一段静止窗口，加两行）。
- 为什么放固件不放主机：陀螺与加速度是**同一采样时刻**的物理量，融合必须在
  采样侧；跨 CAN 到主机再融合会引入 10–20 ms 相位差，加速段正好错拍。

### 5B.4 设计 D2：倾斜有效性位（下位机，约 5 行）

在 `imu_evt`（`0x2033`）里用一个空闲位表示"tilt 当前不可信"（高加速度/冲击保持
期内）。主机据此决定是否用姿态修正，而不是猜。与现有 `IMPACT`（bit0）/
`SATURATED`（bit1）语义并列，不新增对象。

### 5B.5 设计 D3：现场核对与门闩放开（P1，必须做）

核对前 Jetson **不得消费** `0x2031/0x2032`（`canopen_bitfields.md` §7 已有硬规定）。
核对步骤（一次，10 分钟）：

1. 车体水平静止 → 读 `0x2031/0x2032`，期望 0°±0.5°（超出说明零偏/安装角没标）。
2. 车头下压 **10°**（手机水平仪对比） → pitch 变化应为 ∓10°，记录符号。
3. 左侧抬起 **10°** → roll 变化应为 ±10°，记录符号。
4. 0.5 m/s² 匀加速 vs 静止对比同一姿态读数 → 差值应 ≤1°（D1 生效的判据；
   纯加速度解算此时差 ~3°）。
5. 结论回填 `docs/reference/canopen_bitfields.md` §7 + 固件注释，然后放开主机消费。

### 5B.6 主机侧怎么用（与本仓 v1.0 的接口）

| 用途 | 落点 | 阶段 |
|---|---|---|
| IPM 单应按实时 pitch/roll 修正（坡道、加减速） | `rak_road_perception` 的掩码→地面投影（S3 引入） | S3 |
| 冲击/饱和事件 → 丢弃该帧掩码 + 减速 | 掩码发布前的有效性闸 | S3 |
| 里程计独立见证 / 打滑双源判据 | 已有设计（`firmware_capability_spec.md` §7.2） | P2 |
| 时间基准（采样时刻对齐） | ADR-0023，`imu_age` 语义待验收（spec N3） | P1 |

**不新建节点**：v1.0 姿态的唯一消费者是"掩码→地面"那一步，直接在消费点调用；
另起一个编排节点是把一条纯函数调用包成跨进程边界（违反 ADR 的"S1 是直接函数调用"精神）。

### 5B.7 明确不做

- 帧级去模糊、卷帘校正、软件防抖：`vx_max=0.25 m/s`、30 fps 下帧间位移 8 mm，
  收益远低于复杂度。
- 用底盘板 IMU 做航向权威：航向权威是 Jetson 直连 LSM6DSRTR（ADR-0002），本就不依赖。
- 云台：v2.0 若真做视觉增稳，正确形状是全局快门 + 硬触发 + 时间戳同步，
  属于相机选型，不属于底盘板。

---

## 6. 分阶段落地（每步有独立判据）

| 阶段 | 内容 | 判据 |
|---|---|---|
| **S1 感知替换** | TwinLiteNet+ Small → ONNX → trtexec FP16 .plan → `roadseg_node` 换后端 | trtexec ≥50 FPS（Small 实测 59 qps，留余量）；torch 路径移除；`/road_perception/road_mask` 与旧话题名一致；BDD100K 预训练权重在自己场地录 50 帧看边界（不需标注） |
| **S2 定位打通** | 离线雷达建图 + rtabmap localization 模式 + EKF 两层 | rtabmap 纯定位输出位姿与 AprilTag 真值差 <5 cm（静态 30 s）；map→odom 由 rtabmap 独家发布 |
| **S3 控制收口** | nav2.launch.py sensors:=camera 旁路；MPPI 加 TwirlingCritic/小 std | 直道 10 m 往返横向偏差 <5 cm（rtabmap 位姿量）；拐角无 45° 斜走（`/optimal_trajectory` vy 分量检查） |
| **S4 路口/执行** | waypoints.yaml 路口 alias；MCU 死区补偿（对齐 chassis_control.md） | 14 m 多拐角路线成功率 ≥4/5（论文基准口径） |
| **S5 标定闭环** | 在线尺度标定 + EKF 协方差实测回填 | 直行 2 m 回归横向偏移 <5 cm（TODO P1 原判据） |

前置依赖（不做完不许动真车）：
- P0：R76 烧录核验、真机 can0 首帧、TPDO/SYNC 闭合（TODO.md 现状）。
- 感知边界实测：固定场地录相机数据 + 人工看 50 帧掩码，先于 S1 判据。

---

## 7. 风险与未决

| 风险 | 缓解 |
|---|---|
| BDD100K 域到室内固定场地偏移（道路语义差异） | S1 末段做 50 帧人工评估；备选方案 = 在自己场地采集 + 半自动标注 + 微调（开发机有 GPU，训练脚本现成） |
| rtabmap CPU 占用挤占 MPPI/controller_manager 实时性 | Kp/MaxFeatures 起步用默认 500/1000，`DetectionRate` 1 Hz；实测 `/rak/metrics` 延迟后调 |
| 双目标定（双目内参/extrinsics）缺失，stereo_sync 需 rectified 图 | 双目路径若阻塞，降级方案 = rtabmap mono_odometry + IMU（精度换工程量），AprilTag 锚点补偿漂移 |
| 1.3.13 无全向专用 critic | §4.2 的手工 critic 组合；1.3.14 升级列为 P2 |
| Orin Nano 4GB 内存：rtabmap + Nav2 + TRT 引擎同时驻留 | TRT 引擎 ~150 MiB（实测 Peak memory），rtabmap ~200 MiB 级；需实测 `free` 与 swap 行为，超预算先砍 rtabmap 特性（Kp feature 数）再砍帧率 |
| torch 不在系统 Python | 训练/导出移到开发机，板上零 torch 依赖（§2.3 已按此设计） |

---

## 8. 与用户"极其完美"清单的逐条对账

| 用户清单项 | v1.0 落点 | 差距与依据 |
|---|---|---|
| 边界感知损失模型（93.8% mIoU / 2.12M） | TwinLiteNet+ Small（90.6%）+ logits 先上采样再阈值 | 论文级 93.8% 模型无公开代码/权重（GMSANet 无仓库内容），边界质量用后处理补 |
| ORB-SLAM3 纯定位 | RTAB-Map 纯定位 + AprilTag 锚 | 调研：ORB-SLAM3 在 4GB 板工程成本 >收益；rtabmap apt 直装 + landmark 支持 |
| MPPI holonomic:true | MPPI `motion_model: Omni`（1.3.13 实测支持） | `holonomic` 参数名不存在，社区教程误传；wz_max=0 不可行（源码 clip 矛盾） |
| neo_mpc_planner2 | 不采用 | 无 Jazzy 发布/分支，Python 优化器，私有依赖（调研表） |
| 死区比例补偿 | MCU 固件内（对齐 chassis_control.md） | 论文方法可移植；跨 CAN 补偿=开环前馈，错误层 |
| 多阶段 Goal Manager | 不引入（waypoints alias 替代） | 单作者 preprint 实现，侵入决策层；固定地图下 waypoint 方案更简单 |
| 两级闭环里程计标定 | S5（Sensors 26(18):5692 方法，rtabmap 真值） | 论文全文 403，实施前需读全文核实外部真值来源 |
