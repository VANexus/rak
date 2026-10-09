# ADR-0004: 感知算力分配 —— DeepStream 9.1 可用，Holoscan 4.6 不可用，重规划与 GUI 外移

- **状态**: 已接受
- **日期**: 2026-10-05
- **相关**: `docs/architecture.md` §5 · `docs/CONSTRUCTION.md` §2.3 · ADR-0002 · ADR-0005 · ADR-0006 · `TODO.md` P2

---

## 背景

Orin Nano 4GB 的硬约束：

- **统一内存**，CPU/GPU 共用 4 GB（实际可用 ~3.4 GB）。
- **无 DLA/PVA**，**无硬件 H.264 编码器**（Orin NX 才有）→ 任何视频输出回退到软件
  `x264enc`，实测可吃 500%+ CPU。
- 桌面（GNOME 登录态）空闲占 0.8–1.5 GB；`docs/architecture.md` §5 已定原则：
  "不能默认同时常驻所有模型与 GUI"。
- 本机是 **JetPack 7.2 / L4T 39.2.1**。

用户点名要评估 **DeepStream 9.1** 与 **Holoscan 4.6**。

---

## 决策

### D1. DeepStream 9.1 —— **GO**（有约束）

DeepStream 9.1 的官方配对正是 **JetPack 7.2 GA / L4T 39.2**（CUDA 13.2 / TRT 10.16 / GStreamer 1.24），
与本机完全吻合。NVIDIA 未规定最低 RAM，**Orin Nano（含 4GB）本身就是受支持平台**。

必须遵守的约束（否则 4GB 会 OOM 或卡死）：

- **IOU tracker**，不用 NvDCF/DeepSORT（后者吃显存）。
- **headless**（相比桌面省 ~865 MB）。
- **C++ 优先**，Python 次之。
- **关闭 Tiler/OSD**；**不要做 RTSP 回推**（无硬件编码器，软件 x264 会吃满 CPU）。
- 单路相机起步（1 路 detect+track 实测 ~1.2–1.7 GB）。

### D2. Holoscan 4.6 —— **NO**（在本板 + 本 JetPack 上）✅ 2026-10-05 用户确认放弃，不再评估

**决定性事实**：Holoscan 4.6 对 Orin 系列要求 **JetPack 6.2.1（iGPU / CUDA 12）**；
在 JetPack 7 上它只支持 **AGX Thor（dGPU / CUDA 13）**。

→ **DeepStream 9.1 与 Holoscan 4.6 在本板上互斥**：
前者要 JP 7.2，后者要 JP 6.2.1。不可能同时原生运行。

另外，Holoscan 4.6 **没有一等公民的 DeepStream operator**，其推理走 `InferenceOp`/HoloInfer；
与 DeepStream 的桥接只能靠 GStreamer 元素或 HoloHub 社区算子 —— 即"把 DeepStream 当独立管线跑，
再交换 buffer/ROS 消息"，而不是"把 DeepStream 封装成 Holoscan 算子"。

→ 在本设备 + 本目标下，**Holoscan 不推荐**：换来的是主要用不上的抽象层，
代价是放弃 DeepStream 9.1 或降级整个 JetPack。

### D3. 边缘 / 远程分工

| 留在 Orin Nano（4GB） | 外移 x86 工作站（ROS 2 DDS 跨机） |
|---|---|
| 传感器驱动、安全相关实时环 | **MoveIt2 规划**（突发且吃内存；只把 `FollowJointTrajectory` 发回 Jetson） |
| Nav2 局部代价地图 + 控制器 | SLAM 回环/位姿图优化（设备端只跑里程计） |
| 主里程计（轮式 + IMU + 雷达） | DeepStream 二级分类器 / 多路聚合 |
| DeepStream 单路 detect+track | 任何 VLM/大模型（见 ADR-0005） |
| STVL 3D 避障层（见 ADR-0006） | **RViz2 / MoveIt GUI** |

### D4. 同驻上限

**DeepStream + Nav2 + slam_toolbox（headless）是这台机器的天花板。**
再叠 MoveIt2 就没有余量了 —— 预算见下。

---

## 理由

内存预算（headless）：

| 组件 | RAM |
|---|---|
| Ubuntu 24.04 headless + ROS 2 Jazzy 核心 | 500–700 MB |
| DeepStream 单路 detect + IOU track | 1.2–1.7 GB |
| Nav2（controller/planner/BT/costmaps） | 300–500 MB |
| slam_toolbox | 250–400 MB |
| MoveIt2（规划场景 + 小臂） | 250–500 MB |
| **合计** | **~2.5–3.8 GB** |

→ **没有任何余量**。一次消息突发或内存泄漏就 OOM。这就是 D3/D4 的依据。

> ⚠️ **2026-10-05 追加（见 ADR-0008）**：末端（eye-in-hand）摄像头会**再添一路视频流**
> 与一个 ROS 节点。上表已无余量，因此：
> 末端相机**只在抓取时按需启用**，不要与 DeepStream/SLAM 常驻并存；
> 或把它也列入 D3 的"外移"候选。语音模块（TODO P2-6）同理。

---

## 后果

### 正面

- DeepStream 9.1 与现有 JetPack 7.2 **版本天然吻合**，不需要动系统。
- 明确了"Holoscan 要 JP 6.2.1"这个硬冲突，**避免在错误方向投入**。
- 分工表让"哪些节点常驻"可判定，而不是每次靠试。

### 负面（代价）

- **放弃 Holoscan**：若将来确实需要 Holoscan 生态，只能整体降级到 JetPack 6.2.1，
  代价是失去 DeepStream 9.1 与本仓库已适配的 CUDA 13.2 / TRT 10.16 工具链。
- **跨机带来网络依赖**：MoveIt2 远程后，断网则无规划（但底盘/导航/安全仍在本地）。
- **无硬件编码器**：任何需要视频回传的场景都要重新设计（用 ROS 图像话题而非 RTSP）。
- **4GB 是硬顶**：这不是调参能解决的，是选型时就锁死的。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 同时上 DeepStream 9.1 + Holoscan 4.6 | **JetPack 要求互斥**（7.2 vs 6.2.1），本板无法同时原生运行 |
| 为了 Holoscan 降级到 JetPack 6.2.1 | 失去 DeepStream 9.1 与已适配的 CUDA 13.2/TRT 10.16 工具链；且本仓已按 JP 7.2 构建 |
| DeepStream 做 RTSP 回推 | 无硬件编码器，软件 x264 吃满 CPU |
| 全部常驻 4GB（DeepStream + Nav2 + SLAM + MoveIt2） | 预算 2.5–3.8 GB，零余量，必 OOM |
| 把 RViz2/MoveIt GUI 放设备上 | 与相机争 CMA（`docs/CONSTRUCTION.md` §2.3），且吃内存 |

---

## 待闭合项

1. **DeepStream 9.1 实际内存占用需实测**（文档未给最低 RAM，只有单路 ~1.2–1.7 GB 的现场报告）。
2. **已知 DS 9.1 Jetson 问题**：Docker 内 `nvjpegdec` SIGSEGV（workaround
   `GST_PLUGIN_FEATURE_RANK=nvjpegdec:0`）、容器缺 `kmod`、
   Orin Nano 上报告缺 `libgstnvdsseimeta.so` —— 部署前需逐一验证。
3. **跨机 DDS 配置未定**：`ROS_DOMAIN_ID`、QoS 匹配、带宽（点云/图像跨机）。
4. **`nvpmodel` 档位**：DeepStream 满负载下的功耗/热包线未测；本机现为 10W + `jetson-clocks`。
5. **Isaac ROS 4.6.0+ 对 JP 7.2 的 Orin 支持**刚加入，若考虑 `nvblox_nav2` 需先验证（见 ADR-0006）。

---

## 参考

- DeepStream 9.1 安装/发布说明与 JetPack 配对表：<https://docs.nvidia.com/metropolis/deepstream/9.1/text/DS_Installation.html>
- DeepStream 性能与 FAQ（Orin Nano 平台支持、无显式 RAM 下限）：<https://docs.nvidia.com/metropolis/deepstream/dev-guide/text/DS_FAQ.html>
- Holoscan SDK 安装（Orin 需 JetPack 6.2.1；JP 7 仅 Thor）：<https://docs.nvidia.com/holoscan/sdk-user-guide/setup/sdk-installation>
- Jetson 内存优化：<https://www.jetson-ai-lab.com/tutorials/ram-optimization/>
- 本文结论由三路研究中的"DeepStream/Holoscan + 融合"专项得出（2026-10-05）
