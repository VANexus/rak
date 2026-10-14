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

### 3.1 图片流 QoS（best_effort）—— 需与驱动改动协调，暂缓
- 现状：`astra_bridge` 的图片/CameraInfo 发布者用 `create_publisher<Image>(topic, 10)` = **reliable**；
  在仓消费者（`gpu_pointcloud`）已经是 `QoS(1).best_effort().durability_volatile()`。
- **RxO 规则**：reliable 写方 + best-effort 读方 = 兼容（现状可跑）；**反过来（BE 写方 + reliable 读方）不匹配**。
  即：今天不会有事，只有当**某个 reliable 消费者**（如某些 RViz 配置/录制工具）订阅时才会触发重传风暴
  （实测 reliable-reliable 组合 p99 108 ms）。
- 动作：把发布者改成 `rclcpp::SensorDataQoS()`（与 `rak_stereo_driver` 既有约定、ADR-0014 一致）。
  **⚠ 该文件（`astra_bridge_node.cpp`）当前有另一会话的在制品（2026-10-14 15:08）**，
  为避免冲突，此改动**待其落定后合入**。

### 3.2 内核 UDP 缓冲上限持久化 —— ✅ 已实施（2026-10-14）
- 原状：`net.core.rmem_max = wmem_max = 212992`（208 KB）→ 已写入
  `/etc/sysctl.d/99-rak-ipc.conf`：`rmem_max = wmem_max = 16777216`（16 MB），现已生效且开机持久。
- 收益：reliable 场景 p99 108 → 42 ms（实测）；best-effort 场景无副作用。
- 回滚：删该文件 + `sysctl --system`。

### 3.3 CycloneDDS `MaxMessageSize` —— ✅ 已实施（2026-10-14）
- 原状：默认分片很小（61 MB/s 时 8 s 内 3 万余个 UDP 包）。
- 已写入 `cyclonedds.xml`：`<Domain id="47"><Internal><MaxMessageSize>65500</MaxMessageSize></Internal>`
  （**已验证 0.10.5 接受**；`SocketReceiveBufferSize`/`<Socket>` 两种写法均被 schema 拒，勿再试）。
- 实测效果（多次重测，诚实口径）：**UDP 分片/数据包数 −4×**（数据包计数证据）；
  端到端 p50 的收益**落在噪声内**（3.56–3.72 ms 波动，早前单次 2.44 ms 不具复现性）。
  保留理由：分片数下降是结构性收益，无副作用。

### 3.4 彩色流决策 + astra 驱动契约（另一会话在制，本报告不重复动）
- 现状：彩色 92 MB/s 是唯一会真丢帧的档位；`gemini335l.yaml` 注释亦言"无人读"。
- **实测发现（2026-10-14）**：Astra S 上彩色 MJPG（含 320×240）**全部启动失败**
  （`Match openni video mode failed`），HEAD 版驱动因此启动即 abort；且 HEAD 版**深度选了列表第一个
  = 160×120**（过小）。
- **在制进展**：`astra_bridge_node.cpp` 已有另一会话（15:08）的未提交改动：**逐 深度×彩色 组合试启动 +
  全失败时退化为纯深度**（方向与本建议一致，且满足 ADR-0014"深度必修、彩色可选"）。
  → 本报告不改该文件；待其落定后由相机域回归（本仓我方尝试的隔离构建在 CMake 4.4 下 configure 失败，
  未完成真机验证）。

### 3.5 明确不做
- **不换 rmw**（FastDDS 本负载丢帧显著更差）。
- **不动团队 DDS 接口/多播配置**（lo/关多播均无收益）。

## 4. L2 组件化（进行中，2026-10-14）

**已落地**：
- `rak_pointcloud` 的 `gpu_pointcloud_node` 已转为 rclcpp **composable component**
  （构造接受 `NodeOptions` + `RCLCPP_COMPONENTS_REGISTER_NODE`；CMake 同源产出组件库与
  **同名独立可执行**，现有 launch 零改动——已实测独立可执行行为等价）；`package.xml` 补
  `rclcpp_components`；门禁 `check_workspace.py` 通过。
- 新容器入口 `rak_pointcloud/launch/perception_container.launch.py`：
  `component_container_mt` + ComposableNode，`intra:=true|false` 可切（两种模式均实测可加载）。
- **容器端到端实测**（容器内组件 + 外部合成深度流 61 MB/s）：`/camera/depth/points_gpu`
  输出 **24–26 Hz**（1280×800 全分辨率 decimation=1，≈370 MB/s 输出——比输入流重 6 倍，
  抽稀系数 `decimation` 是现成的减压阀）；容器进程 RSS ~372 MB（含 CUDA 上下文）。

**待并入（步骤固定，见 launch 头注释）**：
- `rak_astra_driver/astra_bridge_node.cpp` —— 当前有其他会话在制品，落定后按同一四步组件化。
  **depth 图像→点云将成为第一条真正的容器内零拷贝链路**（届时 61 MB/s 输入 + 370 MB/s
  输出都不再经过序列化/网络栈）；
- `rak_stereo_driver/capture` —— 源已是注册组件，直接加 ComposableNode 即可。

**预期收益（bench 已量化，见 §2 表）**：容器内零拷贝 p50 **0.14–0.18 ms**、零丢帧、CPU 1–3%、
RSS 减半（对照跨进程 p50 2.4–6.4 ms、92 MB/s 丢 40%）。

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

## 7. 本轮实施记录（2026-10-14）

| 项 | 状态 | 落点 |
|---|---|---|
| ② 内核缓冲持久化 | ✅ 已实施 | `/etc/sysctl.d/99-rak-ipc.conf`（16 MB，开机生效） |
| ③ CycloneDDS MaxMessageSize | ✅ 已实施 | `src/rak_bringup/config/cyclonedds.xml`（分片 −4×；延迟增益在噪声内） |
| ① 图片流 QoS | ⏸ 暂缓（在制文件冲突） | 待 `astra_bridge_node.cpp` 的在制品落定后改 SensorDataQoS |
| ④ 彩色/驱动契约 | 🔄 另一会话在制 | 其改动已含"逐组合试 + 纯深度退化"（与本建议一致） |
| **L2 组件化（gpu_pointcloud）** | ✅ 已组件化 + 容器实测 | `rak_pointcloud`（组件库 + 同名可执行 + `perception_container.launch.py`） |
| L2 并入 astra/stereo | ⏳ 待 astra 在制品落定 | 四步配方见容器 launch 头注释 |

回归（61 MB/s 跨进程，②③ 生效后）：**240/240 零丢帧**、p50 3.56 ms、p99 8.85 ms、CPU 8%+5%；
92 MB/s：145/240（**彩色档结构性丢帧，与 ②③ 无关**，根治=组件化零拷贝或关掉彩色）；
单进程 intra：240/240、p50 0.18 ms、CPU 3%。