# IPC / 进程间通信基线（2026-10-10）

> [AI] 本文是 "component 化 + IPC 优化" 的第一阶段（L1 基线）产出：用可复现的合成负载
> 量化当前传输路径的吞吐/延迟/丢帧/CPU，并给出改造清单。工具与配置见
> [`scripts/ipc_bench/`](../scripts/ipc_bench/README.md)。

## 0. 环境与前提

- 设备：Orin Nano **4GB**（总内存 3459 MB，闲时可用 ~1.2 GB；6 核；load ~0.7）
- 中间件：CycloneDDS **0.10.5** + `rmw_cyclonedds_cpp` 2.2.4（域 47），
  `src/rak_bringup/config/cyclonedds.xml` 为最简配置（无共享内存：`libddsc` 未编入 iceoryx）
- 内核 UDP 缓冲上限原为 **212992 B（208 KB）**，已临时提到 16 MB（见 §3.2）
- 负载口径：深度规格 1280×800×16UC1 @30Hz ≈ **61 MB/s**；彩色规格 1280×800×RGB8 @30Hz ≈ **92 MB/s**
  （取自 `gemini335l.yaml`；其注释已指出"没人读的 92 MB/s 拷贝"）

## 1. 结论速览

1. **跨进程 61 MB/s 零丢帧**（CycloneDDS 100% 送达，p50 2.4–3.7 ms，两个进程合计 CPU ~15% 单核）。
2. **跨进程 92 MB/s 开始真丢帧（~40%）**——彩色规格是压垮传输的那一档。
3. **不能换 rmw**：同一 61 MB/s 负载下 FastDDS 丢 **57%**（其 SHM/队列行为对本负载更差），且
   延迟优势仅 ~1.5 ms。**CycloneDDS 是对的选择。**
4. **reliable QoS 白付尾延迟**：同负载下 p99 从 **108 ms → 5.6 ms**（改 best-effort）；
   现役 astra/stereo 图片发布者用的是默认 **reliable**（与 ADR-0014 里 stereo 用 SensorDataQoS 的做法不一致）。
5. **组件化 + 进程内零拷贝的收益已被量化**：全规格 **零丢帧、p50 0.14 ms（低 10–25×）、
   CPU 1%（单进程合计）、RSS 减半**。
6. 传输层小优化：CycloneDDS 加 `<Internal><MaxMessageSize>65500</MaxMessageSize>` 使分片数
   ~4×↓、p50 再降 ~30%（语法已验证；注意 0.10.5 里 `SocketReceiveBufferSize` 的两种写法均被拒）。
7. 内核 UDP 计数器在全部 61 MB/s 用例中 **零丢包**（`RcvbufErrors` 不变）→ 丢包发生在 DDS 层。

## 2. 结果矩阵（修正后口径：先起订阅端，发布端随后）

| 用例（payload / rmw / QoS） | tx | rx | p50 (ms) | p95 | p99 | pub CPU | sub CPU |
|---|---|---|---|---|---|---|---|
| A 61 MB/s · CycloneDDS 现配置 · BE5 | 240 | **240** | 3.68 | 4.28 | 5.44 | 8% | 7% |
| A 61 MB/s · CycloneDDS + T1(64K) · BE5 | 239 | **239** | **2.44** | 3.54 | 5.67 | 7% | 5% |
| A 61 MB/s · FastDDS(SHM) · BE5 | 239 | **103** ✗ | 2.11 | 4.24 | 5.29 | 6% | 2% |
| B 92 MB/s · CycloneDDS 现配置 · BE5 | 240 | **143** ✗ | 6.37 | 11.02 | 12.92 | 13% | 8% |
| B 92 MB/s · CycloneDDS + T1 · BE5 | 240 | **146** ✗ | 5.12 | 8.17 | 12.07 | 10% | 7% |
| B 92 MB/s · FastDDS · BE5 | 240 | **118** ✗ | 3.10 | 6.36 | 7.35 | 10% | 4% |
| A · **单进程 + intra 零拷贝** | 240 | **240** | **0.14** | 0.19 | 0.20 | 1%（合计） | — |

其它单变量实验：
- **深度阶梯（现配置）**：速率 10/15/20/25/30 Hz 下交付率均 ~100%（61 MB/s 内无吞吐拐点）。
- **读/写历史深度（1/5/50 各种组合）**：结果无差异 → 队列深度不是瓶颈。
- **lo 绑定 / 关多播 / 关多播+peer**：无实质收益（±1 Hz、延迟不变）→ 不建议改团队 DDS 接口配置。
- **reliable vs best-effort（61 MB/s）**：丢帧同为 0，**差别只在尾延迟**（p99 108 → 5.6 ms；
  提内核缓冲后 reliable 的 p99 108 → 42 ms，仍显著差于 BE）。

## 3. 立即可做的 L1 改动清单（按收益/风险排序）

### 3.1 图片流改 SensorDataQoS（best_effort + 小深度）—— 建议优先
- 现状：`astara_bridge`（以及部分消费者）用 `create_publisher<Image>(topic, 10)` = **reliable**。
- 收益：p99 尾延迟 −95%（108 → 5.6 ms），消除重传风暴；大图流**不该**用 reliable。
- 风险：低（语义上图像帧允许丢弃；与 ADR-0014 对 stereo 的 SensorDataQoS 约定一致）。
- 验收：`ros2 topic info -v` 显示 Reliability=BEST_EFFORT；重跑本文矩阵 A 组。

### 3.2 内核 UDP 缓冲上限持久化
- 现状：`net.core.rmem_max = wmem_max = 212992`（208 KB）。
- 改动：`/etc/sysctl.d/99-rak-ipc.conf` 写 `net.core.rmem_max=16777216`、`net.core.wmem_max=16777216`（已临时生效）。
- 收益：reliable 场景 p99 108 → 42 ms；BE 场景无副作用（本次已实测）。
- 风险：低（只抬上限，不改默认）。

### 3.3 CycloneDDS 配置加 `MaxMessageSize`
- 现状：默认分片小（61 MB/s 时 61 000+ UDP 包/8 s）。
- 改动：`cyclonedds.xml` 的 `<Domain id="47"><Internal>` 加 `<MaxMessageSize>65500</MaxMessageSize>`
  （**已验证 0.10.5 接受**；`SocketReceiveBufferSize`/`<Socket>` 写法被拒，不要再试）。
- 收益：分片 ~4×↓、p50 3.68 → 2.44 ms。
- 风险：低（纯内部参数）。

### 3.4 彩色流决策 + astra 驱动契约修复（阻断项）
- 现状：彩色 92 MB/s 是唯一会真丢帧的档位；`gemini335l.yaml` 注释亦言"无人读"。
- **新发现（本次实测）**：Astra S 上 `astra_node` **启动即崩**：
  `Start stream failed, OB_SENSOR_COLOR Match openni video mode failed` → 异常 abort。
  根因：驱动**硬性要求彩色+深度同时存在**（`throw std::runtime_error("Astra requires supported color and depth profiles")`），
  且选中的彩色 profile 在 SDK 侧打不开 → 与 ADR-0014 "**深度必修、彩色可选**" 契约不符。
- 建议：① 驱动改为"彩色可失败/可缺省，深度必须成功"（按契约）；② 配置侧默认关闭或降规格彩色。
- 风险：中（碰驱动代码，需相机在场回归）；但**当前彩色路径本来就是坏的**。

### 3.5 明确不做
- **不换 rmw**（FastDDS 本负载丢帧显著更差）。
- **不动团队 DDS 接口/多播配置**（lo/关多播均无收益）。

## 4. L2 组件化（下一步，收益已量化）

把本仓 C++ 节点（`astra_bridge`、`gpu_pointcloud`、`capture` 等）做成 rclcpp **composable
components**，装进 1–2 个 `component_container`（感知容器 / 控制容器），容器内开
`use_intra_process_comms(true)`：

| 指标 | 跨进程（现状） | 进程内零拷贝（目标） |
|---|---|---|
| 交付 | 92 MB/s 丢 40% | **零丢帧（全规格）** |
| p50 延迟 | 2.4–6.4 ms | **0.14 ms** |
| CPU | ~15%（两进程合计）/流 | **1%（合计）** |
| 内存 | 18+20 MB | **16 MB（合一）** |

实施要点：vendor 节点（`orbbec_camera`/`lslidar_driver`）保持独立进程；容器内同进程才免拷贝；
Python 节点（含 1 kHz `spi_imu`）不进容器，单独评估。

## 5. 复现

```bash
# 工具: 复制到独立工作区编译（不在本仓 src/ 下, 避免动 21 包门禁）
mkdir -p ~/ipc_bench_ws/src/ipc_bench && cp -r <repo>/scripts/ipc_bench/* ~/ipc_bench_ws/src/ipc_bench/
cd ~/ipc_bench_ws && source /opt/ros/jazzy/setup.bash && colcon build --packages-select ipc_bench
# 矩阵: 见 scripts/ipc_bench/run_matrix.sh (SECS=8 bash run_matrix.sh)
```

## 6. 方法论教训（自我更正，记录以免复踩）

- 基准工具首版用 **50 µs 轮询自旋**驱动回调 → CPU 数字被污染成 43–45%（真实值 4–8%）。改用
  `SingleThreadedExecutor` 事件驱动后才是真值。
- 首版矩阵**先起发布、1.5 s 后起订阅** → "交付率 81%" 其实只是两个时间窗的重叠比例
  （240 − 1.5×30 = 195）。修正为**先起订阅端**后，61 MB/s 交付率 = **100%**。
  **结论：大流量的"丢帧"结论必须先排除测量窗口错位，再谈传输层。**