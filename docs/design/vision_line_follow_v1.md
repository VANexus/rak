# 纯视觉寻线 v1.0 设计 —— 单目相机 + 几何寻线 + 学习分割可选闸门

> 状态：草案 · 最后更新：2026-11-16

- **日期**: 2026-11-16 · **状态**: 草案（待评审；本文不改任何代码，删除清单见 §2.3）
- **目标**: 只用前视单目 UVC 相机，输出可驱动 `rak_wbc` 的 `/cmd_vel` 寻线指令；
  不依赖雷达、深度相机、双目或地图定位。
- **边界**: 不碰底盘实动、不碰 CAN、不碰 k3s；本文只做设计、审计与删除清单。
- **关联**: [vision_nav_v1.md](vision_nav_v1.md)（现有前视相机 / 标定 / 分割基线）·
  [chassis_control.md](chassis_control.md)（速度盒与低速钳位依据）·
  [cross_host_dds.md](../reference/perf/cross_host_dds.md)（跨机链路实测）·
  [ADR-0011](../adr/0011-wbc-hqp-motion-arbitration.md)（`/cmd_vel` 单写者）·
  [ADR-0023](../adr/0023-time-base-and-alignment.md)（命令按接收时刻计年龄）·
  `src/rak_road_perception`（现有分割节点）· `src/rak_wbc`（下游控制）

---

## 0. 结论摘要

1. **仓库里没有"寻线"实现。** 现有感知只有
   [roadseg_node.py](../../src/rak_road_perception/rak_road_perception/roadseg_node.py)
   的**二类可行驶区域语义分割**（TwinLiteNet+，2024 论文），输出一张全分辨率
   二值掩码；没有任何中心线提取、横向误差、曲率或 `/cmd_vel` 生成逻辑。
   所谓"寻线"需要新建，不是改参数。
2. **两张 2024 论文的模型都在仓里留了痕迹**：LFD-RoadSeg（2024-02）是旧模型遗留，
   TwinLiteNet+（2024-03）是现役主路径。两者都是**自动驾驶道路语义分割**，
   不是"赛道标线"，且全分辨率掩码无法跨 97 Mbit/s 链路——下面逐项列出该删/该降级的位置。
3. **架构结论：感知必须留在 orin，跨机只传紧凑观测量。** 1280×720 bgr8@30 需要
   663 Mbit/s，mono8 掩码也要 221 Mbit/s，都是 97 Mbit/s 预算的 2–7 倍；
   一条 `LineObservation` 约 100 B，30 Hz 只有 0.024 Mbit/s。
4. **方法结论：v1.0 用几何优先（传统 CV），学习分割降级为可选闸门。**
   赛道是固定场地、标线是人为设计的高对比特征；IPM + 扫描线 + 鲁棒拟合 + 时序滤波
   在 CPU 上几毫秒完成，且能给出残差、内点率、有效跨度这些**可观测的失败指标**。
   学习分割保留为"材质变化/无标线"场景的后端与仲裁器，不作为主路径。
5. **删除清单已列出但不执行**（§2.3）。等确认后再动；其中 D2/D3/D4 是 180° 翻转遗留，
   与"相机已物理装正"直接冲突，建议优先收口。

---

## 1. 边界、不变量与单一事实源

### 1.1 不变量

| 编号 | 不变量 | 依据 |
|---|---|---|
| I1 | 感知只用前视单目 UVC；不订阅 `/scan`、`/camera/depth/*`、`/stereo/*` | 本任务定义 |
| I2 | `/cmd_vel` 同一时刻只有一个写者；寻线模式不与 Nav2 同跑 | [ADR-0011](../adr/0011-wbc-hqp-motion-arbitration.md)、`construction.md` |
| I3 | 图像不跨机传输；跨机只传 `LineObservation` 与状态 | §3.3 带宽预算 |
| I4 | 控制器默认 `enabled=false`；无有效观测时发零速 | 对齐 `rak_wbc` 的默认关闭纪律 |
| I5 | 外参未标定（`calibrated=false`）时感知节点拒绝启动，不用"猜的默认值" | 本文 §8 |
| I6 | 跨机时钟不假设同步；新鲜度按本机接收时刻计 | [ADR-0023](../adr/0023-time-base-and-alignment.md) |
| I7 | 源码只在本机改；orin 的 `~/rak` 是部署副本 | 用户 2026-11-16 口径 |

### 1.2 单一事实源（本文不重复定义）

| 事实 | 权威位置 |
|---|---|
| 相机分辨率 / 格式 / 驱动 / 话题 | [camera.launch.py](../../src/rak_bringup/launch/camera.launch.py) |
| 相机内参（fx, fy, cx, cy, k1, k2, p1, p2） | `~/.ros/camera_info/front_camera.yaml`；标定记录见 [vision_nav_v1.md](vision_nav_v1.md) §5A.7 |
| 底盘速度盒 `[0.25, 0.15, 0.5]`、`cmd_timeout=0.3 s` | [node.py](../../src/rak_wbc/rak_wbc/node.py) 的 `PARAM_DEFAULTS` |
| `vx_min=0.02`（低速钳位）与 `vy_max=0.15` | [nav2_params.yaml](../../src/rak_bringup/config/nav2_params.yaml) |
| 死区 / stick-slip 的固件依据 | [chassis_control.md](chassis_control.md) §6 |
| `/cmd_vel` 无 stamp，按接收时刻计年龄 | [ADR-0023](../adr/0023-time-base-and-alignment.md) |
| 跨机 DDS / WiFi 实测 | [cross_host_dds.md](../reference/perf/cross_host_dds.md) |

---

## 2. 现状审计

### 2.1 现有链路的真实形态

**结论：现有实现是"路面分割"，不是"寻线"。** 数据流只有一条：

```
usb_cam (orin, /camera/image_raw, 1280x720 MJPG@30)
  -> roadseg_node (orin, TensorRT FP16, 384x672)
       -> /road_perception/road_mask      mono8 1280x720@30
       -> /road_perception/overlay        bgr8 1280x720@30（仅有订阅者时）
       -> （可选，默认关）nonroad_obstacles / road_clear 点云
```

关键事实（读码 + 现有实测）：

| 事实 | 出处 |
|---|---|
| 分割模型是 TwinLiteNet+ Small（2024-03，BDD100K 可行驶区域 + 车道线双头） | [vision_nav_v1.md](vision_nav_v1.md) §2.1、`roadseg_trt.py` |
| 只取 `da_logits`（可行驶区域），`lane_logits` 注释明确"v1.0 不用" | [roadseg_node.py](../../src/rak_road_perception/rak_road_perception/roadseg_node.py) 文件头、[vision_nav_v1.md](vision_nav_v1.md) §2.2 |
| 单帧 44.5 ms（22.5 FPS，含 1080p→672×384 下采样与 672×384→1080p 概率图放大） | [vision_nav_v1.md](vision_nav_v1.md) §0.1 |
| "上采样比推理还贵" | 同上 |
| 节点 RSS 353–625 MB；全栈含 Laya 时整机只剩 64 MB 可用 | [vision_nav_v1.md](vision_nav_v1.md) §5A.10 |
| 默认 `input_mode=shared_memory`，读 `/dev/shm/cam_frame.bin`；**仓内没有该文件的写者** | `roadseg_node.py` L146–150；全仓 grep 仅命中读端 |
| 实际负载测试必须显式传 `input_mode:=ros_image`、`rotate_180:=true` | [loadtest_full_stack.py](../../scripts/loadtest_full_stack.py) L213–218 |
| 全分辨率掩码 mono8 = 921 600 B/帧；30 Hz = **221 Mbit/s** | 算式见 §3.3 |

**四个结构性缺陷**（与"寻线"目标直接冲突）：

1. **语义不匹配**：BDD100K 的 `drivable area` 是"车能开的道路面"，不是赛道的标线或边界；
   固定场地里的地板纹理、胶带、门框都可能被判成/不判成道路。2026-11-15 的 A/B 已
   实测 LFD v6 把车辆、引擎盖、路缘一起涂成可行驶（室内权重域偏移）。
2. **输出粒度不匹配**：寻线只需要每个前向距离上的一个横向坐标（几打到几十个采样点），
   现在却生成 92 万像素的掩码，还要在 orin 上把它放大回 1280×720。
3. **传输不匹配**：掩码全分辨率 221 Mbit/s，是 97 Mbit/s 预算的 2.3 倍；
   即使降到 640×360 仍有 55 Mbit/s，占掉一半以上链路。
4. **失败不可观测**：分割输出是"置信掩码"，没有几何残差、内点率、有效跨度；
   模型自信地判错时，下游没有任何指标能区分"真路面"和"一整片被涂错的墙"。

### 2.2 2024 年论文引用清单（文件 / 函数 / 方法 / 为什么不该保留）

仓库里能追到 2024 年论文的只有两处，且都已在上游仓库的可执行路径里留下代码或元数据。

#### A. LFD-RoadSeg（arXiv:2402.02430，2024-02）

论文：*Exploiting Low-level Representations for Ultra-Fast Road Segmentation*，
 bilateral 结构、ResNet-18 第一阶段的低层特征主导，面向嵌入式实时道路分割。

| 位置 | 符号 / 方法 | 现在的角色 | 为什么不该保留 |
|---|---|---|---|
| [roadseg_node.py](../../src/rak_road_perception/rak_road_perception/roadseg_node.py) L38–39 | `_MEAN` / `_STD`（ImageNet 均值方差） | 旧 LFD 模型域的归一化常量 | 当前引擎用 `normalize=unit`（`/255`），这两个常量在默认路径上根本不生效；留着只会让下一个人以为分割还吃 ImageNet 归一化 |
| 同上 L76–78、L145、L242–243 | `normalize` 参数与 `imagenet` 分支 | A/B 时切旧模型 | 旧模型已实测 190.8 ms / 5.2 FPS、边界锐度 12.51（TwinLiteNet+ 2.05），且室内权重域不匹配；保留死分支等于保留一条随时会被误开的慢路径 |
| [compare_roadseg.py](../../scripts/compare_roadseg.py) L28–29、L51–73、L120–131 | `_MEAN`/`_STD`、`LfdRunner`、`--lfd-model` | 离线 A/B 工具 | A/B 结论已入档（TODO P1a、vision_nav_v1 §0.1b）；运行时不需要 torch，也不该让 `torch` 出现在任何部署路径的 import 图里 |
| [setup.py](../../src/rak_road_perception/setup.py) L18 | `description="LFD-RoadSeg inference ..."` | 包元数据 | 与现役 TensorRT/TwinLiteNet+ 不符，误导部署与排障 |
| [package.xml](../../src/rak_road_perception/package.xml) L5 | `description="Jetson TorchScript road segmentation..."` | 包元数据 | 板上已无 torch 依赖，TorchScript 路径已删 |
| [README.md](../../README.md) L52 | "Jetson TorchScript 路面分割（LFD-RoadSeg）" | 工程入口 | 同上，事实已过期 |
| [AGENTS.md](../../AGENTS.md) L158 | "TorchScript road segmentation" | 代理规则里的包说明 | 同上 |

#### B. TwinLiteNet+（arXiv:2403.16958，2024-03）

论文：*TwinLiteNet+: A Stronger Model for Real-time Drivable Area and Lane Segmentation*，
BDD100K 双任务（可行驶区域 + 车道线），四档规模。

| 位置 | 符号 / 方法 | 现在的角色 | 为什么不该保留为"寻线主路径" |
|---|---|---|---|
| [roadseg_trt.py](../../src/rak_road_perception/rak_road_perception/roadseg_trt.py) L170–193 | `TrtSegmenter.class_probability` | 对 `da_logits` 做二类 sigmoid/softmax，得到道路概率 | 它输出的是"道路/非道路"两类；没有标线、没有中心线、没有几何量。寻线要的横向误差与曲率仍得另写一层几何，分割只是把问题换了个输入 |
| [roadseg_node.py](../../src/rak_road_perception/rak_road_perception/roadseg_node.py) L221–282 | `_infer_and_publish`：预处理 → 672×384 → 推理 → **概率图放大回 1280×720** → 阈值 0.40 → 发布 mono8 | 现役唯一感知输出 | 放大比推理贵（已实测）；全分辨率掩码 221 Mbit/s 不能跨机；下游真正需要的只是几十个采样点 |
| 同上 L252–255 | `cv2.resize(probability, (image_w, image_h))` 后再阈值 | "边界锐利"补偿 | 对寻线无收益：在 BEV/扫描线空间里拟合，本来就不需要 1280×720 的概率图 |
| 同上 L141–178 | `engine_path`、`road_threshold`、`publish_nav_clouds`、`depth_*`、`max_*_m` 等参数 | 分割 + 深度点云语义过滤 | 深度点云分支依赖 `/camera/depth/points` 与相机外参 TF，直接违反 I1；它属于 v2.0 的 4D 融合，不属于纯视觉 v1.0 |
| 同上 L326–435 | `_pointcloud_callback`、`_organized_xyz`、`_rotation_matrix` | 把分割概率投到深度点云上分"障碍/可通行" | 纯视觉寻线没有深度源；保留它会把一个不存在的传感器依赖带进新链路 |
| [loadtest_full_stack.py](../../scripts/loadtest_full_stack.py) L7、L218 | `rotate_180 on`、全分辨率掩码负载 | 压力测试基线 | 相机已装正，翻转参数是过期事实；压测口径也应改成紧凑观测而不是全图 |

**该保留的部分**（不要一起删）：

- `roadseg_trt.py` 的 `TrtSegmenter` 是**可复用库**：引擎加载、IO 绑定、ctypes CUDA 拷贝
  都已实测跑通。v1.0 把它作为可选 `segmentation` 后端复用，不重写。
- TwinLiteNet+ 的 `.plan` 引擎与导出脚本：作为"材质变化 / 无标线"场景的候选后端保留。
- `roadseg_node.py` 的 `road_threshold` 语义（道路类概率阈值）在分割后端里继续有效。

**一句话结论**：LFD-RoadSeg 是**该删的遗留**；TwinLiteNet+ 是**该降级的工具**——
它不该是寻线主路径，但它的 TensorRT 管线值得作为可选闸门保留。

### 2.3 准备删除的清单（等确认，不执行）

> 说明：用户口径是"相机已物理装正、180° 翻转逻辑刚删掉"，但**当前工作树与 HEAD 里
> 仍能看到翻转路径**（已逐处核对）。下面 D2–D4 用于把这件事收口；如果别处已删，
> 这份清单就是残留检查表。

| ID | 文件 | 符号 / 行 | 动作 | 前置 / 风险 |
|---|---|---|---|---|
| **D1** | `roadseg_node.py` | `_MEAN`/`_STD`（L38–39）、`normalize` 参数（L76–78、L145）、`imagenet` 分支（L242–243） | 删除；只保留 `unit` 归一化 | 确认没有 ImageNet 归一化引擎在跑（当前引擎为 `unit`） |
| **D2** | `roadseg_node.py` | `rotate_180` 参数（L155）、`cv2.rotate(..., ROTATE_180)`（L234–238） | 删除 | 先删调用侧 D4；否则 `--ros-args -p rotate_180:=true` 会变成未声明参数直接报错 |
| **D3** | `camera.launch.py` | `FLIPPED_FRAME`（L47）、`flip` 参数（L84–90）、`image_flip_node`（L137–153）、文件头"UPSIDE DOWN"（L18–23） | 删除翻转节点与相关参数；相机已装正，`camera_info` 也在正向帧标定 | 删前确认标定 yaml 是正向帧产物；`image_flip_node` 本来就不转发 `camera_info`，删掉是净收益 |
| **D4** | `loadtest_full_stack.py` | 文件头 L7、`-p rotate_180:=true`（L218） | 删除该参数与注释 | 无；先于 D2 |
| **D5** | `setup.py` / `package.xml` / `README.md` / `AGENTS.md` | L18 / L5 / L52 / L158 | 改为"TensorRT 二类可行驶区域分割（可选后端）" | 纯文案，无风险 |
| **D6** | `compare_roadseg.py` | `LfdRunner`（L51–73）、`_MEAN`/`_STD`（L28–29）、`--lfd-model`、文件头 L4 | A/B 结论已入档后删除 LFD 路径，只保留 TwinLite/TRT 路径 | 不影响运行时；如需复现历史 A/B 再保留一个 tag |
| **D7** | `roadseg_node.py` | 文件头"TorchScript inference node"（L1–11）、类 docstring（L42） | 改文案，不删逻辑 | 无 |
| **D8** | `roadseg_node.py` | `input_mode=shared_memory` 默认值（L146–150） | 改为 `ros_image`，或确认外部 feeder 后保留 | 仓内无写者；默认值目前等于"起了但永远收不到帧" |
| **D9** | `roadseg_node.py` | `publish_nav_clouds` 与 `_pointcloud_callback`（L107–139、L284–435） | **本次不删**；默认关，并声明属 v2.0 | 纯视觉 v1.0 不启用；删除会破坏 v2.0 的深度语义路径 |
| **D10** | `vision_nav_v1.md` | §0.1b A/B 表、§2.4 归一化表述、§5A.7 翻转段 | **不删历史**，加"已被本文取代/仅历史记录"注记 | 文档是记录，不是代码；只标状态 |

**明确暂不动**：`roadseg_trt.py`、`.plan` 引擎、TwinLiteNet+ 权重、`scripts/export_twinlite_onnx.py`、
`scripts/build_roadseg_engine.py`、`rak_wbc`、CAN 相关任何文件。

---

## 3. 系统架构

### 3.1 部署边界

| 位置 | 跑什么 | 理由 |
|---|---|---|
| **orin**（`192.168.5.215`，驱动与传感器） | `usb_cam` + `line_detect_node`（几何寻线；可选 TRT 分割后端） | 图像不出板；全分辨率掩码/图像不过 WiFi |
| **本机**（ROS 2 上层逻辑） | `line_follow_controller`（控制律 + 状态机 + 看门狗）→ `/cmd_vel` → `rak_wbc` | 控制与仲裁留在本机；接收的只是 100 B 级观测 |
| **不涉及** | 雷达、深度、双目、k3s、CAN | 本任务边界 |

### 3.2 数据流

```
┌──────────────────────── orin ────────────────────────┐        ┌──────────── 本机 ────────────┐
│ usb_cam 1280x720 MJPG@30 ──▶ /camera/image_raw        │        │                              │
│                                 │                     │        │                              │
│                     line_detect_node                  │        │                              │
│   预处理 → IPM(BEV) → 扫描线/鲁棒拟合 → 时序滤波       │        │                              │
│                                 │                     │        │                              │
│        /line_perception/observation (≤128 B, 30 Hz) ──┼──WiFi──▶ line_follow_controller       │
│        /line_perception/status      (1 Hz, JSON)      ─┼────────▶   │ 状态机 + 看门狗          │
│        /line_perception/debug_image (可选, 2 Hz)      ─┼────────▶   ▼                          │
│                                                       │        │ /cmd_vel (Twist, 30 Hz)      │
└───────────────────────────────────────────────────────┘        │   ▼                          │
                                                                 │ rak_wbc (100 Hz 仲裁)         │
                                                                 │   ▼                          │
                                                                 │ mecanum_drive_controller ...  │
                                                                 └──────────────────────────────┘
```

**关键点**：

- 图像只在 orin 内部走 DDS（同机可用共享内存传输；v1.0 不额外引入 intra-process 复杂度）。
- 跨机只有 `LineObservation`（约 100 B）与低频状态；`debug_image` 默认关。
- `/cmd_vel` 由控制器独占；寻线 launch 不启动 Nav2。

### 3.3 带宽预算（97 Mbit/s 为设计上限）

用户口径：本机 ↔ orin 5 GHz WiFi，单向上限约 **97 Mbit/s**（2026-11-16）。
仓库曾实测 TCP 407 Mbit/s、`ping` avg 1.4 ms（[cross_host_dds.md](../reference/perf/cross_host_dds.md)），
但**设计按 97 Mbit/s 保守取值**，不拿历史峰值当预算。

| 数据形态 | 每帧字节 | 30 Hz 带宽 | 占 97 Mbit/s | 判定 |
|---|---:|---:|---:|---|
| 1280×720 bgr8 | 2 764 800 | **663.6 Mbit/s** | 684% | 不可跨机 |
| 1280×720 mono8（现有掩码） | 921 600 | **221.2 Mbit/s** | 228% | 不可跨机 |
| 640×360 mono8 | 230 400 | 55.3 Mbit/s | 57% | 勉强，仍不建议 |
| 1280×720 JPEG q75（估 100–150 KB） | ~100–150 K | 24–36 Mbit/s | 25–37% | 可跑但抖动/解码成本高 |
| **LineObservation（本设计）** | **~100 B** | **0.024 Mbit/s** | **0.025%** | **推荐** |
| debug JPEG 320×180 @2 Hz | ~8–12 K | 0.13–0.19 Mbit/s | 0.2% | 仅调试，默认关 |

**结论**：把检测放在 orin 不是"优化"，是**唯一满足带宽预算的架构**。
任何"本机收图再算"的方案都会在 97 Mbit/s 上直接失败或吃满链路。

### 3.4 时间与新鲜度

- `LineObservation.header.stamp` 用**相机帧时间戳**（orin 时钟），只用于离线对齐与日志。
- 控制器**不比较跨机时间戳**；收到消息时记本机 `time.monotonic()`，用
  `age = now_local - receive_monotonic` 判断新鲜度（对齐 ADR-0023 对 `/cmd_vel` 的口径）。
- 消息里携带 `latency_ms`（检测端：发布时刻 − 图像时间戳，同机时钟），
  用于定位"是检测慢还是链路慢"；它不参与控制判决。
- 观测 QoS：`best_effort`、`keep_last 1`。控制量是易腐数据，不重传旧帧；
  丢包表现为 `age` 变大 → 看门狗降级，这是可观测且安全的。

### 3.5 `/cmd_vel` 单写者

- `line_follow.launch.py` **不包含** Nav2 / collision_monitor / velocity_smoother。
- 控制器启动时检查 `/cmd_vel` 现有发布者：若已有非本节点的发布者，打印错误并保持
  `enabled=false`（不抢写）。实现用 `Node.get_publishers_info_by_topic('/cmd_vel')`。
- 需要与 Nav2 共存时，v1.1 再引入带优先级与超时的 mux；v1.0 不做第二套仲裁
  （ADR-0011 已把常规运动仲裁权给 `rak_wbc`，不要在它前面再加一层）。

---

## 4. 感知设计

### 4.1 输入契约

| 项 | 值 | 来源 |
|---|---|---|
| 话题 | `/camera/image_raw`（`sensor_msgs/Image`，`bgr8`） | `camera.launch.py`（`usb_cam`，`/camera` 命名空间） |
| 分辨率 | 1280×720 | 同上；内参按此分辨率标定，**不缩放** |
| 帧率 | 标称 30 Hz（实测 29.4–29.7 Hz） | [vision_nav_v1.md](vision_nav_v1.md) §5A.6 |
| 内参 | fx=844.40, fy=851.46, cx=614.20, cy=401.18 | `~/.ros/camera_info/front_camera.yaml`；用户确认仍有效 |
| 畸变 | k1=+0.2428, k2=−1.4443, k3=0, p1=+0.0011, p2=−0.0001 | 同上 |
| 帧同步 | 只用图像；`camera_info` 只用于启动时读取内参或校验 | 不依赖精确时间同步 |
| 输入健康 | `age`（帧间隔）、饱和度、模糊度、帧尺寸/编码 | 见 §4.9 |

**不订阅**：`/scan`、`/camera/depth/*`、`/stereo/*`、`/odom`、`/tf`（感知层不需要位姿）。

### 4.2 预处理

按顺序执行，全部在 orin CPU：

1. **ROI 裁剪**：只保留图像下方的前向地面区域。起点 `roi_top`、终点 `roi_bottom`
   由安装俯仰决定，标定后写入参数；默认按"下倾 20–30°、下沿 0.4–0.5 m、上沿约 4 m"
   的既有建议给出**起点值**（[vision_nav_v1.md](vision_nav_v1.md) §5A.5），必须用
   §8.2 的实测结果替换。
2. **缩放**：缩到 `working_width=640`（保持 16:9），降低 CLAHE、IPM 与扫描线的成本。
   缩放不是精度瓶颈：最终拟合在 BEV 的 0.02 m 栅格上做。
3. **光度归一化**：转灰度后用 CLAHE（`clipLimit=2.0`、`tile=8×8`）压逆光/阴影的
   大尺度亮度差；**不做逐帧自动 gamma**（现有 `adaptive_gamma` 会把帧间亮度差变成
   检测抖动，见 §2.3 的清理建议）。
4. **质量掩码**：
   - 过曝：`V > 250` 的像素占比 `sat_frac`；
   - 镜面反光：HSV 中 `S > 40 且 V > 245` 的占比 `specular_frac`；
   - 模糊：ROI 灰度 Laplacian 方差（按对比度归一）`blur_score`。
   三者不直接改像素，只作为置信度与降级判据（§4.9）；反光/过曝像素在拟合时降权。
5. **曝光策略**：标定后**锁定曝光与增益**（若 V4L2 支持），避免自动曝光的帧间振荡
   被误判成标线亮度变化。该相机的曝光控制能力**未验证**，列入 §11 待办。

### 4.3 地面投影（IPM）

单目没有深度，但**平坦地面 + 已知相机位姿**足以把图像映射到鸟瞰图（BEV）。
这正是"纯视觉"能给出**米制**横向误差的原因，也是外参必须实测的原因。

**坐标与符号（唯一约定，后文全部引用本节）**：

- `base_link`：x 向前、y 向左、z 向上（REP-103）。
- `camera_optical_frame`：z 向前、x 向右、y 向下。
- 相机在 `base_link` 下的位置 `(x_c, y_c, h)`，姿态 `(roll φ, pitch θ, yaw ψ)`；
  光轴下倾时 `θ < 0`（绕 +y 轴）。
- BEV 栅格：`X ∈ [x_min, x_max]`（前向，m），`Y ∈ [−y_half, +y_half]`（左正，m），
  分辨率 `res=0.02 m`；默认 `X ∈ [0.20, 4.00]`、`Y ∈ [−1.00, 1.00]` → 190×100。
- 曲线上一点 `(X, Y)` 的横向误差 `e = Y`（+ 表示目标在车中心线左侧）；
  航向误差 `ψ_e = atan(dY/dX)`（+ 表示目标朝左偏）。
- 控制器符号：`e > 0` 或 `ψ_e > 0` → `wz > 0`（逆时针/左转）。

**单应**：地面平面 `z=0` 下，图像点 `p` 到 BEV 点 `P=(X,Y)` 的映射是 3×3 单应 `H`：

```
P_h = H · p_h,   H = K · [r1 r2 t]   （r1,r2 为 R_cb 的前两列，t = −R_cb·[x_c,y_c,h]^T）
```

实现上不手推 `r1/r2`，而是：

1. 在 BEV 上取 4 个角点（如 `(0.5,±0.5)`、`(3.5,±0.5)`）；
2. 用相机模型把它们投到图像（`cv2.projectPoints`）；
3. `cv2.getPerspectiveTransform` 求 `H`，再用 `cv2.warpPerspective` 生成 BEV。

这样 `H` 与内参/外参的符号错误会在"四角投影是否落在 ROI 内"这一步立刻暴露，
而不是在控制端表现为方向反了。

**误差预算**（沿用 [vision_nav_v1.md](vision_nav_v1.md) §5A.2 的算式）：

| 俯仰误差 Δθ | 1 m 处横向误差 | 2 m 处 | 3 m 处 |
|---|---:|---:|---:|
| 0.5° | 0.9 cm | 1.7 cm | 2.6 cm |
| 1.0° | 1.7 cm | 3.5 cm | 5.2 cm |

所以外参标定目标定为 **±0.5°**；寻线真正在意的 0.5–2 m 区间误差 ≤1.7 cm，
与现有底盘验收的 5 cm 横向偏差同一量级，足够。

**假设与失效**：IPM 假设地面平坦。坡道、门槛、地毯边缘会让 BEV 距离失真；
v1.0 不处理坡道（固定场地），只用 §4.9 的 `span/residual` 置信度在失真时降速。

### 4.4 候选方法对比：传统 CV vs 学习

| 维度 | 传统几何 CV | 学习分割（TwinLiteNet+ / 自训） | 混合（本设计） |
|---|---|---|---|
| 适合场景 | 有可见标线/边界、材质稳定、光照可控 | 无标线、材质多变、背景复杂 | 默认几何；材质突变时用分割仲裁 |
| 冷启动成本 | 0 标注，参数可解释 | 需本场地采数 + 标注/微调 | 先上几何，数据够了再接学习 |
| orin 计算 | 2–5 ms CPU，<200 MB | 17–33 ms GPU + 353–625 MB | 几何常开；分割按需/降帧 |
| 逆光/阴影 | 依赖 CLAHE + 梯度特征，需调参 | 数据覆盖到时更稳 | 几何降级时切分割/降速 |
| 反光 | 可显式掩掉高光 | 可能整片判错且自信 | 高光掩码 + 鲁棒拟合 |
| 材质变化 | 换极性/阈值，可能失效 | 训练覆盖到时最稳 | 置信度下降 → 减速/切换 |
| 失败可观测性 | 残差、内点率、有效跨度 | 基本没有 | 几何指标为主 |
| 可验收性 | 可用合成图/离线回放定量 | 需要标注真值 | 几何指标可单独验收 |
| 结论 | **v1.0 主路径** | **可选后端/闸门** | **采用** |

**为什么不是"直接上学习"**：现有 TwinLiteNet+ 是 BDD100K 道路语义，不是赛道标线；
要它做寻线，仍然要在掩码上做几何（边界提取 + 拟合 + 误差），等于多付 17–33 ms 和
数百 MB 内存，却没有减少几何工作量。**为什么不是"只上传统"**：一旦赛道换成无标线、
强反光或深色材质，几何会集体失效；保留学习后端是给这类场地留退路。

### 4.5 v1.0 主路径：几何寻线

#### 4.5.1 两种赛道模式

| 模式 | 定义 | 检测量 | 适用 |
|---|---|---|---|
| `guide_line` | 地面上有一条可见引导线（深色/浅色/彩色胶带） | 单条线的横向位置 | 循迹赛、贴线行驶 |
| `track_corridor` | 赛道由左右边界围出，中心没有线 | 左/右边界各一条，中心线取中点 | 有边界的赛道/场地 |

两种模式共用同一套 BEV、响应图、拟合与观测计算，只是"每行取几个峰"不同。
**模式选择是评审必须确认的第一件事**（§11 Q1）：如果赛道既没有线也没有连续边界，
本设计的主路径不成立，需要改为学习后端为主。

#### 4.5.2 响应图（对标线的"像不像"打分）

在 BEV 灰度图上逐像素计算**局部对比度响应**，而不是绝对灰度：

- `dark`（深色线）：`resp = max(0, local_mean − I)`，用大核黑帽（black-hat）；
- `bright`（浅色线）：`resp = max(0, I − local_mean)`，用大核顶帽（top-hat）；
- `color`（彩色线）：在 LAB 的 a/b 或 HSV 的 H/S 上算到标定色中心的距离，取反。

大核均值（如 31×31）把缓变的阴影/逆光梯度减掉，这是"阴影不产生假线"的关键；
绝对阈值（Otsu/固定阈值）在逆光下会整体漂移，不采用。

#### 4.5.3 扫描线与候选点

- 沿 BEV 的 X 方向每隔 `scan_step=0.05 m` 取一行（默认 0.30–3.00 m，共 55 行）。
- 每行在 Y 方向找 1–2 个显著峰（`min_prominence` 由响应图的 90 分位自适应）。
- `guide_line`：取离上一帧预测位置最近的峰；没有预测时取最强峰。
- `track_corridor`：以预测中心为界，左右各取一个峰；左右宽度与上一帧的
  `track_width` 偏差超过 30% 时判该行无效（防左右边界串线）。
- 每行候选带一个权重：响应强度 × （1 − 该像素的过曝/反光惩罚）。

#### 4.5.4 鲁棒拟合

- 模型：`Y(X) = c0 + c1 X + c2 X² + c3 X³`（三次足够覆盖直道、S 弯与定曲率弯）。
- 求解：RANSAC（阈值 `max_residual=0.03 m`，迭代 50 次）选内点，再用 Huber 加权
  最小二乘精修；`track_corridor` 先分别拟合左右边界，再取中点曲线。
- 输出：`c0..c3`、内点率 `inlier_ratio`、残差 RMS `residual_m`、有效跨度 `span_m`
  （最远连续有效行的 X）。
- 若内点率 `< 0.5` 或 `span_m < 0.8 m`，该帧判为低置信（不直接丢，交给时序层）。

#### 4.5.5 时序滤波

- 状态：`[e, ψ_e]`（横向误差、航向误差），匀速模型 + 观测噪声由拟合残差给出。
- 线性 Kalman，30 Hz；`Q` 取"0.2 m/s 横向扰动"量级，`R` 取 `residual_m²`。
- 新息门：`|e_meas − e_pred| > 0.35 m` 或 `|ψ_meas − ψ_pred| > 0.35 rad` 时，
  该帧降为 `CAUTION` 并只做预测外推，不直接更新（防串线/误检把车带偏）。
- 遮挡/丢帧时最多外推 `0.4 s`，之后进入 `HOLD`（§5.5）。

#### 4.5.6 观测量计算

在 `s_look` 处：

```
Y_s   = c0 + c1 s + c2 s² + c3 s³
ψ_s   = atan(c1 + 2 c2 s + 3 c3 s²)
κ_s   = (2 c2 + 6 c3 s) / (1 + (c1 + 2 c2 s + 3 c3 s²)²)^(3/2)
e     = Y_s
```

`s_look = clamp(k_s · v, s_min, s_max)`，默认 `k_s=3.0 s`、`s_min=0.8 m`、`s_max=2.5 m`；
速度越低看得越近（低速时近处精度更高）。这是**起点值**，按 §9.6 的验收调。

### 4.6 可选后端：学习分割作为闸门

不改变主路径，只在两种情况下启用：

1. **几何置信度持续低**（连续 10 帧 `< 0.5`）：用分割掩码给出"可通行先验"，
   把落在掩码外的几何候选点剔除后重拟合；
2. **赛道材质切换**（内点率突降）：用掩码判断"是真没线，还是线的外观变了"。

实现约束（重要）：

- 复用 [roadseg_trt.py](../../src/rak_road_perception/rak_road_perception/roadseg_trt.py)
  的 `TrtSegmenter`，**不复制**引擎加载代码；
- 在 672×384 模型分辨率上取边界，**不放大回 1280×720**；把边界点直接投到 BEV
  的同一坐标系做拟合；
- 分割后端与几何后端输出同一种 `TrackFit`，上层无感；
- 分割后端默认 `enabled=false`；开启时每帧额外 17–33 ms GPU，需重新核对内存预算。

### 4.7 鲁棒性设计

| 现象 | 机理 | 对策 | 可观测指标 |
|---|---|---|---|
| 逆光 | 整体过曝/对比度压缩 | CLAHE；用局部对比度而非绝对灰度；过曝像素降权；必要时切分割后端 | `sat_frac`、`blur_score` |
| 阴影 | 大尺度亮度阶跃产生假边 | 大核黑帽/顶帽减掉低频；色度特征（阴影保色度、变亮度）；鲁棒拟合 | `inlier_ratio`、`residual_m` |
| 镜面反光 | 高光饱和，线在反光处断裂 | HSV 高光掩码剔除；Huber 降权；时序外推跨过短暂缺口 | `specular_frac`、`span_m` |
| 赛道材质变化 | 颜色/纹理/极性改变 | 三种极性（dark/bright/color）按段配置；置信度下降时减速；必要时分割闸门 | `inlier_ratio` 突降 |
| 运动模糊/振动 | 麦轮辊子高频振动 | 机械减振（既有建议）；Laplacian 方差门；必要时降速 | `blur_score` |
| 相机自动曝光振荡 | 帧间亮度跳变被当成线 | 标定后锁曝光/增益（能力待验证）；帧间归一化 | 帧间 `e` 抖动 |
| 标线被遮挡/磨损 | 有效跨度变短 | `span_m` 门限；近处优先；保持上一帧短时外推 | `span_m` |
| 弯道过急 | 三次多项式拟合外推失真 | 限制 `s_look ≤ 2.5 m`；曲率超限降速 | `κ_s`、残差 |

**不做的**：HDR 合成、去模糊反卷积、卷帘校正、多帧超分。理由：30 fps、0.18–0.25 m/s
下帧间位移只有几毫米，收益低于复杂度（对齐 [vision_nav_v1.md](vision_nav_v1.md) §5B.7）。

### 4.8 失败检测指标

| 指标 | 定义 | 正常 | 注意 | 失败 |
|---|---|---|---|---|
| `age_ms` | 控制器收到观测的年龄 | < 60 ms | 60–200 ms | > 200 ms |
| `inlier_ratio` | 拟合内点行 / 有效候选行 | > 0.75 | 0.5–0.75 | < 0.5 |
| `residual_m` | 内点残差 RMS | < 0.02 | 0.02–0.03 | > 0.03 |
| `span_m` | 连续有效前向跨度 | > 1.5 m | 0.8–1.5 m | < 0.8 m |
| `sat_frac` | ROI 过曝像素占比 | < 0.10 | 0.10–0.25 | > 0.25 |
| `specular_frac` | ROI 高光像素占比 | < 0.05 | 0.05–0.15 | > 0.15 |
| `blur_score` | 归一化 Laplacian 方差 | > 阈值（标定） | — | < 阈值 |
| `innovation_m` | 与 KF 预测的差 | < 0.15 m | 0.15–0.35 m | > 0.35 m |

阈值全部是**起点值**，必须用 §9 的离线回放与故障注入标定后写死；不接受"拍脑袋默认值"
直接上车。

---

## 5. 控制设计

### 5.1 接口契约

| 方向 | 话题 / 服务 | 类型 | 频率 | 说明 |
|---|---|---|---|---|
| 入 | `/line_perception/observation` | `rak_interfaces/LineObservation` | 30 Hz | 感知观测量 |
| 入 | `/wbc/status` | `std_msgs/String`（JSON） | 10 Hz | 只在 `enabled=true` 且 `mode∈{nav,servo}` 时放行 |
| 出 | `/cmd_vel` | `geometry_msgs/Twist` | 30 Hz | `linear.x` 前向，`linear.y=0`，`angular.z` 左正 |
| 服务 | `/line_follow/enable` | `std_srvs/SetBool` | 按需 | 默认 `false`；开启前必须过 §5.5 的自检 |
| 出 | `/line_follow/status` | `std_msgs/String`（JSON） | 5 Hz | 状态机、指标、拒绝原因 |

**超时**：控制器 `obs_timeout=0.2 s` 发零速；`rak_wbc` 的 `cmd_timeout=0.3 s` 是第二道
保险（它会把超时命令当零速，见 `node.py` `_tick`）。

### 5.2 控制律

采用"曲率前馈 + 横向/航向反馈"，比纯 pure-pursuit 更容易在低速调稳：

```
s     = clamp(k_s * v_prev, s_min, s_max)
e     = obs.lateral_error_m          # + 左
psi   = obs.heading_error_rad        # + 左
kappa = obs.curvature_1pm            # 路径曲率前馈

v = v_ref * (1 - min(0.8, k_ve*|e| + k_vpsi*|psi|))
v = clamp(v, v_min, v_max) * speed_scale(state)

w = v * kappa + k_e * e + k_psi * psi
w = clamp(w, -wz_max, wz_max)
```

符号检查：`e>0`（目标在左）→ `w>0`（左转），与 §4.3 的约定一致。
`v*kappa` 是弯道前馈，让直道比例项不必承担全部转向。

**起点增益**（仅起点值，按 §9.6 调）：`v_ref=0.18 m/s`、`v_min=0.03`、`v_max=0.25`、
`k_ve=2.0 1/m`、`k_vpsi=1.5 1/rad`、`k_e=2.0 rad/(s·m)`、`k_psi=2.5 1/s`、`wz_max=0.5 rad/s`。
`v_max` 与 `wz_max` 不是新事实，直接引用 `rak_wbc` 的 `base_vmax`；`v_min` 取
`nav2_params.yaml` 的 `vx_min=0.02` 之上一个最小余量，避免落到 stick-slip 区。

### 5.3 全向取舍：v1.0 为什么 `vy=0`

麦轮能横移，但现有文档已记录：45° 斜走只驱动两个轮子，实车会滑
（[vision_nav_v1.md](vision_nav_v1.md) §0.4，PR #6460 实证）。
寻线的核心是把车头对准赛道；用 `wz` 纠航向、`vx` 前进即可，**不需要横移**。
v1.0 固定 `linear.y=0`，把全向能力留给 v1.1 的小偏差修正（`|vy| ≤ 0.05 m/s`，
且必须有实测打滑数据支撑）。

### 5.4 限幅、斜率与死区

- 速度盒：`vx ∈ [0, 0.25]`、`vy = 0`、`wz ∈ [−0.5, 0.5]`（引用 `base_vmax`）。
- 斜率：`|Δvx| ≤ 0.4 m/s²`、`|Δwz| ≤ 2.0 rad/s²`；30 Hz 下即每帧
  `Δvx ≤ 0.0133`、`Δwz ≤ 0.0667`。防止观测跳变直接变成电机冲击。
- 死区：不在控制器里补底盘死区。死区是 PWM→轮速的非线性，补偿归 MCU 固件
  （[chassis_control.md](chassis_control.md)）；控制器只保证 `vx` 不长期停在
  stick-slip 区（`v_min` 已处理）。

### 5.5 状态机与降级阶梯

| 状态 | 进入条件 | 动作 | 退出 |
|---|---|---|---|
| `NORMAL` | 观测有效且 `confidence ≥ 0.7`、`age ≤ 60 ms` | 正常速度 | 任一指标变差 |
| `CAUTION` | `0.4 ≤ confidence < 0.7` 或 `age 60–200 ms` 或新息门触发 | 速度 ×0.6；用 KF 外推；继续尝试 | 连续 5 帧恢复 → `NORMAL` |
| `HOLD` | 无有效观测 0.2–1.0 s，或 `confidence < 0.4` | `vx=0, wz=0`，30 Hz 持续发布零速 | 连续 5 帧有效 → `CAUTION` |
| `LOST` | 无有效观测 > 1.0 s | 零速；置 `degraded`；**不自动恢复** | 人工 `/line_follow/enable` 重新使能 |

- 任何状态都**不后退**（`vx ≥ 0`）；倒车不在寻线语义里。
- `LOST` 不自动恢复：与 `rak_wbc` 的 `degraded` 纪律一致，避免"画面恢复但车已经偏出赛道"
  时突然自己冲出去。
- 降级优先于"保持上一帧"：外推最多 0.4 s，绝不用 1 s 前的线继续开。

### 5.6 看门狗

```
on_timer(1/control_hz):
    if not enabled or not wbc_ready: publish_zero(); return
    age = now - last_obs_recv
    if age > obs_timeout: publish_zero(); state = HOLD/LOST; return
    ... 控制律 ...
    publish(cmd)
```

- `wbc_ready`：`/wbc/status` 的 `enabled=true` 且 `mode∈{nav,servo}`；`mode=idle` 时
  控制器只发零速（不试图靠 `/cmd_vel` 绕过 WBC 的状态机）。
- 进程退出、异常、Ctrl-C：由 `finally` 发一次零速；但**不依赖**退出路径保证安全，
  `rak_wbc` 的 0.3 s 超时与底盘控制器的 `reference_timeout` 才是兜底（ADR-0019 口径）。

### 5.7 `rak_wbc` 的 TF 前置（容易漏的集成点）

`rak_wbc._tick` 每周期先 `lookup_transform("map", "base_link")`；**拿不到就发零速**
（`node.py` L384–389）。也就是说：寻线控制器即使 `/cmd_vel` 发得对，只要没有
`map → base_link`，WBC 也不会动。

v1.0 的三个选项：

| 选项 | 做法 | 适用 | 代价 |
|---|---|---|---|
| **A（推荐）** | 现有 EKF（轮式里程计 + IMU，`ekf.yaml`）发布 `odom→base_footprint`；`map→odom` 用静态 0 | 真机寻线 | 位姿会漂，但寻线只用观测量、不用地图位姿；WBC 的 `support_margin` 仍有意义 |
| B | `world_frame:=base_link`，让 `lookup_transform` 退化为单位阵 | 台架/无运动自检 | `support_margin` 变成原点处的值，不能用于稳定性判断；不用于实车 |
| C | 跑完整定位栈（rtabmap/AprilTag） | 不需要 | 与"纯视觉寻线"目标无关，徒增依赖 |

**结论**：v1.0 选 A。它不引入雷达/深度，只复用底盘已有的轮速 + IMU；
`map→odom` 是静态 0 这一事实要写进 launch 注释，避免被误认为"定位好了"。

---

## 6. 数据结构与接口契约

### 6.1 新消息：`rak_interfaces/msg/LineObservation.msg`

```text
# 一帧寻线观测。发送端 orin，接收端本机。
# 符号约定见 docs/design/vision_line_follow_v1.md §4.3（唯一约定，勿在别处重定义）。

uint8 STATUS_OK=0
uint8 STATUS_CAUTION=1
uint8 STATUS_HOLD=2
uint8 STATUS_LOST=3

uint8 SOURCE_CLASSICAL=0
uint8 SOURCE_SEGMENTATION=1
uint8 SOURCE_HYBRID=2

std_msgs/Header header

uint8 status              # STATUS_*
uint8 source              # SOURCE_*
uint8 valid               # 0/1，冗余但便于快速判断

float32 confidence        # 0..1
float32 lateral_error_m   # 前视点 Y，+ 左
float32 heading_error_rad # 前视点航向，+ 左
float32 curvature_1pm     # 前视点曲率
float32 lookahead_m       # 计算该观测所用的 s
float32 track_width_m     # corridor 模式下的左右边界宽度；guide_line 为 0
float32 valid_span_m      # 有效前向跨度
float32 residual_m        # 拟合残差 RMS
float32 inlier_ratio      # 0..1
float32 latency_ms        # 检测端发布时刻 - 图像时间戳（同机时钟）

float32[4] center_coeffs  # Y(X)=c0+c1X+c2X²+c3X³，BEV 坐标，单位 m
float32[4] left_coeffs    # corridor 模式左边界；guide_line 全 0
float32[4] right_coeffs   # corridor 模式右边界；guide_line 全 0
```

**契约**：

- 所有长度单位 m，角度 rad，曲率 1/m，时间 ms。
- `header.stamp` 是**图像帧时间戳**；`header.frame_id = "base_link"`（观测量已在该系）。
- `center_coeffs` 的 X 是 BEV 前向距离（不是图像行号），Y 左正。
- `status != STATUS_OK` 时 `valid=0`，但**保留最后一次可用的观测值**供外推/日志，
  不要填 NaN（DDS 与下游都更难处理）。
- 字段只增不改语义；新增字段必须更新本文 §6.1 与消息注释。

### 6.2 话题与 QoS

| 话题 | 类型 | 方向 | QoS | 频率 | 备注 |
|---|---|---|---|---|---|
| `/camera/image_raw` | `sensor_msgs/Image` | orin 内部 | `sensor_data`（best effort） | 30 Hz | 不跨机 |
| `/line_perception/observation` | `rak_interfaces/LineObservation` | orin → 本机 | best effort, keep_last 1 | 30 Hz | 约 100 B |
| `/line_perception/status` | `std_msgs/String` | orin → 本机 | reliable, keep_last 1 | 1 Hz | JSON：指标、后端、耗时 |
| `/line_perception/debug_image` | `sensor_msgs/CompressedImage` | orin → 本机 | best effort, keep_last 1 | 2 Hz（可调） | 默认关，仅调试 |
| `/line_follow/status` | `std_msgs/String` | 本机 | reliable, keep_last 1 | 5 Hz | 状态机 + 拒绝原因 |
| `/cmd_vel` | `geometry_msgs/Twist` | 本机 | reliable, keep_last 1 | 30 Hz | 单写者 |

### 6.3 参数表（感知）

| 参数 | 默认 | 单位 | 依据 / 说明 |
|---|---:|---|---|
| `image_topic` | `/camera/image_raw` | — | `camera.launch.py` |
| `process_hz` | 30.0 | Hz | 相机标称；实测 29.4–29.7 |
| `camera_info_url` | `""` | — | 空则从 `camera_info` 话题读；必须非空或可读 |
| `working_width` | 640 | px | 起点值；在 720p ROI 上做 2× 降采样 |
| `roi_top` | 0.45 | ratio | **起点值**，由 §8.2 俯仰实测替换 |
| `roi_bottom` | 0.98 | ratio | 同上 |
| `bev_x_min` | 0.20 | m | 起点值；近处不贴车头 |
| `bev_x_max` | 4.00 | m | 起点值；与 FOV 上沿一致（§8.2 核定） |
| `bev_y_half` | 1.00 | m | 起点值；赛道半宽 + 余量 |
| `bev_res` | 0.02 | m/px | 2 cm 栅格；与 §4.3 误差预算匹配 |
| `camera_height_m` | — | m | **无默认，必须实测**（§8.2） |
| `camera_pitch_deg` | — | deg | **无默认，必须实测**（§8.2） |
| `camera_roll_deg` | — | deg | **无默认，必须实测**（§8.2） |
| `camera_x_m` / `camera_y_m` | — | m | **无默认，必须实测**（§8.2） |
| `calibrated` | `false` | bool | `false` 时节点拒绝启动（I5） |
| `line_mode` | `guide_line` | — | `guide_line` / `track_corridor`；§11 Q1 确认 |
| `line_polarity` | `dark` | — | `dark` / `bright` / `color`；§8.3 标定 |
| `line_color_lab` | `[]` | — | `color` 极性时使用；§8.3 标定 |
| `scan_step_m` | 0.05 | m | 55 行覆盖 0.3–3.0 m |
| `min_span_m` | 0.8 | m | 失败门限起点值 |
| `max_residual_m` | 0.03 | m | RANSAC 阈值起点值 |
| `min_inlier_ratio` | 0.50 | — | 失败门限起点值 |
| `fit_order` | 3 | — | 直道/S 弯/定曲率弯足够 |
| `kf_q_lateral` | 0.04 | m²/s | 起点值；按 §9.2 回放调 |
| `kf_r_scale` | 1.0 | — | `R = (scale·residual)²` |
| `innovation_gate_m` | 0.35 | m | 起点值；按误检率调 |
| `max_extrapolate_s` | 0.4 | s | 超过即 HOLD |
| `use_segmentation` | `false` | bool | 可选后端 |
| `engine_path` | `""` | — | 空则不加载 TRT |
| `debug_image` | `false` | bool | 默认关（带宽） |
| `debug_hz` | 2.0 | Hz | 调试流上限 |

### 6.4 参数表（控制）

| 参数 | 默认 | 单位 | 依据 / 说明 |
|---|---:|---|---|
| `observation_topic` | `/line_perception/observation` | — | §6.2 |
| `cmd_vel_topic` | `/cmd_vel` | — | `rak_wbc` 默认 |
| `control_hz` | 30.0 | Hz | 与观测同频；WBC 100 Hz 再平滑 |
| `enabled` | `false` | bool | 默认关闭 |
| `require_wbc_enabled` | `true` | bool | 读 `/wbc/status` |
| `v_ref` | 0.18 | m/s | 起点值；低于 0.25 上限，留调参余量 |
| `v_min` | 0.03 | m/s | 高于 `vx_min=0.02`，避开 stick-slip |
| `v_max` | 0.25 | m/s | 引用 `base_vmax[0]` |
| `k_ve` | 2.0 | 1/m | 起点值（速度随横向误差下降） |
| `k_vpsi` | 1.5 | 1/rad | 起点值 |
| `k_e` | 2.0 | rad/(s·m) | 起点值 |
| `k_psi` | 2.5 | 1/s | 起点值 |
| `k_s` | 3.0 | s | 前视距离随速度；起点值 |
| `s_min` / `s_max` | 0.8 / 2.5 | m | 起点值；`s_max` 受曲率与拟合阶数限制 |
| `wz_max` | 0.50 | rad/s | 引用 `base_vmax[2]` |
| `ax_max` | 0.40 | m/s² | 起点值；按电机与验收调 |
| `alpha_max` | 2.0 | rad/s² | 起点值 |
| `obs_timeout_s` | 0.20 | s | 起点值；小于 WBC 0.3 s |
| `caution_age_s` | 0.06 | s | §4.8 表 |
| `lost_age_s` | 1.00 | s | §5.5 |
| `recover_frames` | 5 | — | 防抖 |
| `use_vy` | `false` | bool | §5.3 |
| `vy_max` | 0.05 | m/s | 仅 `use_vy=true` 时生效；起点值 |

### 6.5 标定文件：`data/calib/front_camera_extrinsics.yaml`

```yaml
# 唯一事实源：相机安装外参与赛道几何。
# calibrated=false 时 line_detect_node 拒绝启动。
calibrated: false
date: ""
method: "ground control points"      # 见 design §8.2
camera:
  height_m: 0.0        # TODO(measured)：光学中心到地面
  pitch_deg: 0.0       # TODO(measured)：下倾为负
  roll_deg: 0.0        # TODO(measured)
  x_m: 0.0             # TODO(measured)：相对 base_link 前向偏移
  y_m: 0.0             # TODO(measured)：相对 base_link 左向偏移
track:
  mode: "guide_line"   # guide_line | track_corridor
  width_m: 0.0         # TODO(measured)：corridor 模式必填
  line_width_m: 0.0    # TODO(measured)：guide_line 模式参考值
  polarity: "dark"     # dark | bright | color
```

**规则**：不允许在代码里给外参"看起来合理"的默认值；`calibrated=false` 必须硬失败。
这比"猜一个 0.20 m 高度先跑起来"安全得多，也符合本仓"魔数必须有实测依据"的纪律。

---

## 7. 模块划分与代码落点

### 7.1 新包 `rak_line_perception`（部署在 orin）

```text
src/rak_line_perception/
  package.xml
  setup.py
  config/line_perception.yaml
  launch/line_perception.launch.py
  rak_line_perception/
    __init__.py
    node.py                 # ROS 壳：参数、订阅、发布、诊断、生命周期
    camera_input.py         # Image -> bgr8；尺寸/编码校验；帧率统计
    preprocess.py           # ROI、缩放、CLAHE、过曝/高光/模糊指标
    ground_projection.py    # 内参+外参 -> H；BEV warp；四角自检
    track_model.py          # 三次多项式、s 处 e/psi/kappa、置信度
    temporal.py             # 2 状态 KF + 新息门 + 外推
    detectors/
      __init__.py
      base.py               # Detector 协议：detect(bev, prior) -> TrackFit
      classical.py          # 响应图、扫描线、RANSAC/Huber 拟合
      segmentation.py       # 可选：TrtSegmenter -> 边界 -> TrackFit
  test/
    test_ground_projection.py
    test_classical_detector.py
    test_temporal.py
    test_contract.py
```

职责边界：

- `node.py` 只做 I/O 与编排，**不含算法**；算法在 `detectors/` 与 `track_model.py`，
  可以纯 numpy 单测。
- `segmentation.py` 复用 `rak_road_perception.roadseg_trt.TrtSegmenter`；
  跨包依赖写进 `package.xml`，不复制代码。
- 所有外参从 `data/calib/front_camera_extrinsics.yaml` 读；节点不内置默认值。

### 7.2 新包 `rak_line_follow`（部署在本机）

```text
src/rak_line_follow/
  package.xml
  setup.py
  config/line_follow.yaml
  launch/line_follow.launch.py
  rak_line_follow/
    __init__.py
    node.py                 # ROS 壳：订阅、服务、定时器、状态发布
    controller.py           # 控制律（纯函数，可单测）
    guard.py                # 状态机、看门狗、WBC 放行、单写者检查
    cmd_vel.py              # 限幅、斜率、零速发布
  test/
    test_controller.py
    test_guard.py
    test_contract.py
```

- `controller.py` 的输入是一个 `LineObservation` 的纯数据视图，输出 `(vx, vy, wz)`；
  不 import ROS，方便离线回归。
- `guard.py` 是安全层，任何"观测过期/低置信/WBC 未使能/已有别的 `/cmd_vel` 写者"
  都在这里拦截。

### 7.3 `rak_interfaces` 增量

- 新增 `msg/LineObservation.msg`（§6.1）；
- `CMakeLists.txt` 的 `rosidl_generate_interfaces` 加一行；
- 不新增服务/动作，避免接口面膨胀。

### 7.4 明确不新建的东西

| 不做 | 原因 |
|---|---|
| `twist_mux` / 第二套速度仲裁 | `rak_wbc` 已是唯一常规运动写者（ADR-0011）；寻线模式与 Nav2 互斥即可 |
| 新的 EKF / 新的定位 | 寻线不用地图位姿；TF 复用现有 EKF（§5.7） |
| 新的相机驱动 | `usb_cam` 已实测 29.4 Hz；MJPG 路径不需要重写 |
| 在 orin 上编辑源码 | 源码只在本机改，部署是复制（I7） |
| 把整张图/掩码跨机 | 带宽预算不允许（§3.3） |

---

## 8. 标定流程

### 8.1 C0：相机内参（已完成）

- 事实源：`~/.ros/camera_info/front_camera.yaml`，fx=844.40、fy=851.46、cx=614.20、cy=401.18。
- 标定分辨率必须等于运行分辨率 1280×720；换分辨率必须重标（内参不缩放）。
- 复核判据：`camera_info` 实际发布值与之逐字段一致；不一致先查 `camera_name` 与 yaml 路径。

### 8.2 C1：安装外参（本设计的前置，未做）

目标：`height_m`、`pitch_deg`、`roll_deg`、`x_m`、`y_m`，俯仰 ±0.5°、位置 ±1 cm。

步骤（一次，约 30 分钟）：

1. **尺量**：相机光学中心到地面的垂直距离 `height_m`；到 `base_link` 的前向/左向偏移
   `x_m`、`y_m`。卷尺即可，记录测量点（镜头前端还是 CMOS 面）。
2. **地面控制点**：在车前地面沿车体中心线贴 3 个标记，距离分别 0.5 / 1.0 / 2.0 m；
   再在 1.5 m 处左右各贴一个 ±0.5 m 的标记。
3. **俯仰求解**：先用尺量值 + 目测俯仰填初值，把 5 个标记投到 BEV；调 `pitch_deg`
   使标记的 BEV X 与真值误差最小（建议做一维搜索，步长 0.1°）。
4. **横滚求解**：调 `roll_deg` 使左右 ±0.5 m 标记的 BEV Y 对称。
5. **验收**：5 个控制点的 BEV 重投影误差 RMS < 1.5 cm；俯仰拟合的残差曲线在 ±0.5°
   内平坦（否则说明高度或镜头畸变模型有问题）。
6. **落盘**：写 `data/calib/front_camera_extrinsics.yaml`，`calibrated: true` + 日期 + 方法。

**为什么不用消失点**：消失点法对长直边界的检测误差敏感，且需要一条足够长的直线；
地面控制点法直接、可复核，且能同时验证 `roll` 与横向偏移。

### 8.3 C2：线的极性与阈值（按赛道分段）

1. 在赛道每段（直道/弯道/不同材质）录 30 帧，车辆静止或手推（不开发动机命令）。
2. 对每段跑三种极性（`dark`/`bright`/`color`），看 `inlier_ratio` 与 `span_m`；
   选最好的极性，记录该段的 `line_polarity` 与阈值。
3. 若不同段极性不同，v1.0 先人工切配置；v1.1 再做按 `span_m`/残差自动切换。
4. 验收：每段 ≥95% 帧 `valid=1`，且 `residual_m < 0.02`。

### 8.4 C3：控制增益（只在仿真/台架，不动真车）

1. 用离线回放把 `LineObservation` 序列喂给 `controller.py`，记录 `vx/wz`。
2. 阶跃：`e` 从 0 跳到 0.15 m，要求超调 < 20%、调节时间 < 1.5 s（0.18 m/s）。
3. 正弦：`e = 0.1 sin(2π·0.2t)`，要求相位滞后 < 0.15 s、幅值衰减 > 0.7。
4. 斜率/限幅：确认 `Δvx/Δwz` 不超 §5.4；确认 `LOST` 时一帧内归零。
5. **真车调参必须等底盘运动许可**（本任务边界）；在此之前只做离线与台架。

### 8.5 标定产物的版本管理

- 外参 yaml 与赛道配置进 `data/calib/`（按仓库惯例 gitignored 的模型/标定目录），
  但在文档里记录**日期 + 方法 + 验收数字**；
- 每次改外参必须重跑 §9.2 的离线回放，指标变化写进提交说明；
- 不允许"只改 yaml 不重验"。

---

## 9. 测试与验收

### 9.1 单元测试（无硬件）

| 测试 | 断言 |
|---|---|
| `test_ground_projection.py` | 合成针孔相机 + 已知地面点，BEV 重投影误差 < 1 mm；四角投影落在 ROI |
| `test_classical_detector.py` | 合成 BEV：已知三次曲线 + 噪声/阴影/缺口，拟合 `e` 误差 < 2 cm |
| `test_temporal.py` | 连续丢帧 10 帧：外推 ≤ 0.4 s，之后 HOLD；新息门拒掉 0.5 m 跳变 |
| `test_controller.py` | 阶跃/正弦指标；限幅/斜率；`vx≥0` |
| `test_guard.py` | 过期、低置信、WBC 未使能、已有写者 → 零速/拒绝 |
| `test_contract.py` | 消息字段、单位、符号、参数默认值与本文一致 |

### 9.2 离线回放（有数据，无运动）

- 工具：`scripts/replay_line_follow.py`（新增，读图片目录或 rosbag → 跑检测 → 输出
  `LineObservation` 序列 + 叠加图 + 指标 CSV）。
- 数据：固定场地录 50–100 帧（不同光照/材质段），人工在 BEV 上标 3 个前向距离的
  中心点作为弱真值（不需要像素级标注）。
- 指标：
  - `e` 的 RMSE（0.5/1.0/2.0 m 三处）；
  - `valid` 帧率、`inlier_ratio`/`residual_m`/`span_m` 分布；
  - 误检率（无标线帧被报 `valid=1` 的比例）。
- 判据：标称光照下 `e` RMSE < 2 cm、`valid` ≥ 95%、误检 < 1%。

### 9.3 无硬件门禁（合成图像 → 观测 → `/cmd_vel`）

新增 `scripts/verify_line_follow.py`（对齐现有 `verify_4d_fusion.py` 的风格）：

1. 起一个合成图像发布者（画一条已知多项式曲线，含阴影/缺口）；
2. 起 `line_detect_node`（`calibrated=true` 的测试外参）；
3. 断言 `/line_perception/observation` 出现，`e` 与合成真值误差 < 2 cm；
4. 起 `line_follow_controller`（`enabled=false` → 服务打开）；
5. 断言 `/cmd_vel` 出现且 `vx>0`、`wz` 符号正确；
6. 停掉图像发布者 → 200 ms 内 `/cmd_vel` 变零；1 s 后状态 `LOST`；
7. 再发 5 帧有效图 + 服务重新使能 → 恢复 `NORMAL`。

**这条门禁不需要相机/底盘**，是 v1.0 合入前的硬门槛。

### 9.4 故障注入

| 注入 | 期望 |
|---|---|
| 全黑/全白帧 | 200 ms 内零速；状态 `HOLD`→`LOST` |
| 强过曝（30% 像素 255） | `CAUTION` + 降速；不误报 `valid=1` 高速行驶 |
| 高光斑（镜面） | 反光区被降权；拟合仍收敛或 `CAUTION` |
| 图像延迟 300 ms | 100 ms 内降速，200 ms 内零速 |
| 观测 topic 中断 | 同上 |
| 曲线跳变 0.5 m | 新息门拒绝，不发对应转向 |
| `/cmd_vel` 已有别的发布者 | 控制器拒绝使能，打印错误 |
| WBC `enabled=false` | 控制器发零速，不抢写 |

### 9.5 资源与带宽

| 项 | 判据 |
|---|---|
| `line_detect_node` CPU | < 25% 单核（720p@30，经典路径） |
| `line_detect_node` RSS | < 200 MB（不含可选 TRT 后端） |
| 端到端延迟 | p95 < 50 ms（图像时间戳 → `/cmd_vel`） |
| 观测带宽 | < 0.1 Mbit/s（`ros2 topic bw`） |
| 跨机图像 | 0 字节（`debug_image=false` 时） |
| 全栈内存 | 加入本设计后仍留 ≥ 300 MB 可用（对齐现有 2.0 GB 口径） |

### 9.6 真机验收（本次不做，需底盘运动许可）

| 项 | 判据 | 口径 |
|---|---|---|
| 直道 10 m 往返 | 横向偏差 < 5 cm | 与现有 MPPI 验收同口径 |
| S 弯（曲率半径 ≥ 1.0 m） | 全程 `valid` ≥ 95%，无 `LOST` | 新增 |
| 14 m 多拐角路线 | 成功率 ≥ 4/5 | 与 [vision_nav_v1.md](vision_nav_v1.md) §6 的论文基准口径一致 |
| 逆光段 | 不 `LOST`，速度自动降 | 现场 |

**在获得许可前，以上一律写"未验证"。**

---

## 10. 实施里程碑

| 阶段 | 内容 | 判据 | 是否碰真车 |
|---|---|---|---|
| **M0 骨架** | 新消息 + 两个包骨架 + 参数/标定 schema + 单测 | `colcon build` + `pytest` 绿；`check_docs.py` 绿 | 否 |
| **M1 几何感知** | ROI/CLAHE/IPM/响应图/扫描线/鲁棒拟合/KF | 离线回放：`e` RMSE < 2 cm、`valid` ≥ 95% | 否 |
| **M2 控制闭环** | 控制律 + 状态机 + 看门狗 + `/cmd_vel` | `verify_line_follow.py` 7 项全过；故障注入全过 | 否 |
| **M3 真机低速** | 直道/缓弯，0.10–0.18 m/s | 直道横向偏差 < 5 cm | **是**（需许可） |
| **M4 鲁棒性** | 逆光/阴影/反光/材质；可选分割闸门 | 现场各段 `valid` ≥ 95%，无 `LOST` | 是 |
| **M5 验收** | 14 m 多拐角路线 | 成功率 ≥ 4/5 | 是 |

依赖：

- M1 依赖**固定场地录数**（相机已在 orin，可先录不动机器人的静态帧）；
- M3 起依赖底盘运动许可与 `map→base_link` TF 方案落地（§5.7 A）；
- 可选分割后端（§4.6）不阻塞 M0–M3，可并行。

---

## 11. 风险与未决

| 编号 | 问题 | 影响 | 处置 |
|---|---|---|---|
| Q1 | **赛道到底是"一条线"还是"左右边界"？** | 决定 `line_mode` 主路径；两者都没有则本设计不成立 | 评审第一项确认；先录 50 帧看实际画面 |
| Q2 | 相机高度/俯仰/横滚未实测 | IPM 比例全错，横向误差不可信 | §8.2，M1 前置；未标定节点拒绝启动 |
| Q3 | 相机自动曝光/增益能力未验证 | 帧间亮度跳变 → 检测抖动 | 用 `v4l2-ctl --list-ctrls` 核实；能锁就锁 |
| Q4 | 5 GHz WiFi 实际 97 Mbit/s 与历史 407 Mbit/s 不一致 | 若按 407 设计，可能超预算 | 按 97 保守设计；观测只有 0.024 Mbit/s，不受影响 |
| Q5 | 麦轮横移打滑 | 若用 `vy` 纠偏会滑 | v1.0 `vy=0`；v1.1 需实测再开 |
| Q6 | `rak_wbc` 需要 `map→base_link` | 无 TF 则 WBC 发零速，车不动 | §5.7 A：轮式+IMU EKF + 静态 `map→odom` |
| Q7 | 学习后端与 Laya 抢内存 | 全栈只剩 64 MB 可用 | 默认关；开启前重跑内存预算 |
| Q8 | 赛道材质强反光/无标线 | 几何失效 | 分割闸门 + 降速；必要时改学习为主（需重新评审） |
| Q9 | 标定/参数"看起来合理"的默认值 | 错误被静默接受 | 外参无默认 + `calibrated=false` 硬失败；其余参数标"起点值"并纳入验收 |
| Q10 | 与 Nav2 同时运行 | 双 `/cmd_vel` 写者 | 启动检查 + launch 互斥；v1.1 再考虑 mux |

**评审通过后**：按 `docs/README.md` 的流程落 ADR（下一个编号 **0029**），
再把本文状态从"草案"改为"已落 ADR"；删除清单（§2.3）单独提交、逐项验证。

---

## 12. 附录

### A. 符号表

| 符号 | 含义 | 单位 / 符号约定 |
|---|---|---|
| `X` | BEV 前向距离 | m，车头前方为正 |
| `Y` | BEV 横向距离 | m，左正 |
| `e` | 横向误差 `Y(s)` | m，+ 左 |
| `ψ_e` | 航向误差 | rad，+ 左（逆时针） |
| `κ` | 曲率 | 1/m，+ 左弯 |
| `s` | 前视距离 | m |
| `vx, vy, wz` | 车体速度 | m/s, m/s, rad/s；`wz` + 左转 |
| `θ` | 相机俯仰 | deg，下倾为负 |
| `φ` | 相机横滚 | deg |
| `h` | 相机高度 | m |

### B. 带宽算式

```
B = W × H × C × fps            （bit/s，C=每像素字节数×8）
1280×720×3×8×30 = 663.6 Mbit/s
1280×720×1×8×30 = 221.2 Mbit/s
640×360×1×8×30  =  55.3 Mbit/s
LineObservation ≈ 100 B × 8 × 30 = 0.024 Mbit/s
```

### C. 参考

- LFD-RoadSeg：*Exploiting Low-level Representations for Ultra-Fast Road Segmentation*，
  arXiv:2402.02430（2024-02）。
- TwinLiteNet+：*TwinLiteNet+: A Stronger Model for Real-time Drivable Area and Lane
  Segmentation*，arXiv:2403.16958（2024-03）。
- IPM / 单应：Hartley & Zisserman, *Multiple View Geometry*，第 13 章（平面诱导单应）。
- Pure pursuit / Stanley：Coulter (1992)、Thrun et al. (2006)；本文只用其几何直觉，
  控制律按 §5.2 的曲率前馈 + 反馈实现。
- ROS REP-103（坐标轴与单位）。
