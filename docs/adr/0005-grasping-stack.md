# ADR-0005: 抓取栈分三阶段推进 —— 几何优先，学习型次之，VLM/大模型远程

- **状态**: 已接受（阶段划分）；具体实现选型 **待阶段 1 实测后定**
- **日期**: 2026-10-05
- **2026-10-05 修订回退**：同日曾按"末端是**吸嘴**"修订本文（改用平面吸附几何 + SuctionNet），
  但用户随后更正末端为**舵机夹爪（即平行夹爪）**，故**该修订作废，本文恢复原样**。
  → **反极点 / 平行夹爪范式成立**：GraspNet-1Billion、GPD、自写反极点检测
  **均可照原方案使用**；SuctionNet 相关方案不采用。见 [ADR-0008](0008-end-effector-and-perception.md)。
- **相关**: `src/rak_astra_driver` · `src/rak_moveit_config` · `src/rak_arm_simulation` · `brain/` · `TODO.md` P3 · ADR-0004（算力分配）

---

## 背景

硬件与约束：

- **Orin Nano 4GB，统一内存**（CPU/GPU 共用 4 GB）。无 DLA/PVA，**无硬件 H.264 编码器**
  （软件 x264 会吃满 CPU）。桌面空闲占 0.8–1.5 GB，headless 剩 ~2.5–3 GB。
- 臂是**六定位关节 + 独立夹爪**，但当前只有软件夹具（`rak_arm_simulation`），
  几何/限位是测试占位值，实机门槛见 `hardware_acceptance.md` §Arm Unlock Gates。
- 传感器：Orbbec Astra 深度 + USB 双目 + N10P 2D 雷达。
- **`pcl_ros` 当前未安装**（已核实），`pcl_conversions` 已装。

---

## 决策

**分三阶段推进，每阶段的选型由上一阶段的实测结果决定，不提前锁死。**

| 阶段 | 内容 | 跑在哪 | 前置 |
|---|---|---|---|
| **1. 几何抓取** | 点云 → 体素/直通/欧氏聚类 → 反极点候选 → MoveIt2 IK 过滤 → 执行 | **边缘** | P2-3 点云管线 |
| **2. 学习型抓取** | GraspNet-1Billion（非商业许可）作服务，处理杂乱/未知物体 | **边缘**（1–3 Hz） | 阶段 1 链路通 |
| **3. 语言/VLM 与高端网络** | GroundingDINO+MobileSAM 选目标；AnyGrasp 或更大模型 | **远程 x86**（或边缘降频） | 局域网 ROS 2 跨机 |

**明确不采用**：
- `moveit_grasps` —— 在 Jazzy 上**未维护且被 blocked/unreleased**，不用。
  改用 `moveit_task_constructor`（Jazzy 维护中）做 pick/place 时序编排。
- 任何 VLA 类模型（π0 / GR00T / SmolVLA）**在 4 GB 上都不成立**
  （SmolVLA 450M 的底线是 **8 GB**，π0 ~3B 连 8 GB 都装不下）。
- 把抓取网络与 Nav2/SLAM/DeepStream **同时常驻**在 4 GB 上（见下）。

---

## 理由

### 为什么几何优先

- **零显存、零许可证风险、全在设备上**。GPD（BSD）是 C++/PCL，**纯 CPU 可跑**，
  工作站约 5 Hz，Nano 的 A78AE 上更低但可用；自写反极点检测只需几百行
  （法线估计 → 反极点对 → 工作空间/IK 过滤）。
- **先把"点云 → 抓取位姿 → MoveIt2 执行"整条链路打通**，再换更好的生成器。
  链路不通时换网络只会把问题复杂化。
- 当前臂是占位几何，**用重网络去拟合占位模型没有意义**。

### 为什么学习型放第二、且优先 GraspNet 而非 AnyGrasp

| | GraspNet-1Billion | AnyGrasp |
|---|---|---|
| 许可 | **非商业**（可自由非商用） | **仅研究**，**按机器指纹锁定**，有到期日；商用需与 SJTU MVIG 谈 |
| 依赖 | PyTorch + 自定义 CUDA 算子 | **MinkowskiEngine 0.5.4**（稀疏 3D 卷积） |
| JetPack 7 适配 | 需按本机 JetPack 重编算子 | ME 针对旧 PyTorch/CUDA 11 时代，**CUDA 13 上移植成本高** |
| TensorRT 加速 | 可 | **稀疏卷积不支持 TRT**，无标准加速路径 |
| 体积/显存 | ~10 MB 权重，**<1 GB** | SDK+模型 ~2 GB |
| 速度 | ~1–3 Hz（Nano，随点数变化） | ~8 Hz（Orin 级，2025 综述） |

→ **先 GraspNet**（许可宽松、显存小、无 ME 移植坑）；AnyGrasp 只在"研究用途 + 愿意啃
MinkowskiEngine"时才上。

### 为什么 VLM/大模型远程

- GroundingDINO-Swin-T + SAM-ViT-B 量化后**或许**能在 4 GB 上跑低个位数 Hz，
  但**没有任何已公开的 4 GB Orin Nano 部署**（公开的都是 8 GB Nano / 16 GB NX / AGX）。
- 点云本身很小（20–40 K 点），**跨机传点云/图像 + 回传掩码**在局域网上 100–300 ms，
  对"抓取"这个任务速率完全够。
- 与 `brain/` 的约束一致：S2 延迟 12–30 s，**本就不能进实时环**（`docs/explanation/architecture.md` §4）。

---

## 后果

### 正面

- 阶段 1 就能端到端"导航到物体 → 抓取"，**零许可证风险**，且全部在设备上。
- 分阶段意味着**每步都有可回退点**：阶段 2/3 出问题可以退回阶段 1。
- 许可风险被隔离：非商业的 GraspNet 只用在阶段 2，商用需替换时只换一个服务节点。

### 负面（代价）

- 几何方法在**杂乱堆叠**场景成功率明显低于学习型 → 阶段 1 的天花板较低，是刻意的。
- 远程 VLM 引入**网络依赖**；断网时语言抓取不可用（但几何/学习型仍在本地）。
- `moveit_grasps` 不能用，pick/place 时序要自己用 TMC 搭，**多一份维护**。
- GraspNet 是**非商业许可**：一旦项目转向商用，阶段 2 必须整体替换。

---

## 被否决的方案

| 方案 | 否决理由 |
|---|---|
| 直接上 AnyGrasp | 许可按机器锁定且仅研究用途；MinkowskiEngine 在 CUDA 13/ARM 上移植成本高；无 TensorRT 路径 |
| 用 `moveit_grasps` | Jazzy 上未维护、blocked/unreleased |
| 直接上 VLA 端到端 | **4 GB 装不下任何 VLA**（SmolVLA 底线 8 GB） |
| 抓取网络与 Nav2/SLAM/DeepStream 同时常驻 4 GB | 预算算不过来（见 ADR-0004）：headless ~1 GB + DeepStream ~1.2 GB + slam_toolbox ~0.6 GB + Nav2 ~0.4 GB ≈ 2.8 GB，**还没算 CUDA 上下文、相机缓冲、DDS、tf** |
| 把整张大场景的稠密 3D 地图常驻内存 | 4 GB 放不下。**全局定位、局部建模、用完即弃** |

---

## 待闭合项

1. **`pcl_ros` 未安装**（`ros-jazzy-pcl-ros` 2.6.5 已为 Jazzy 发布）。
2. **点云管线不存在**：`rak_astra_driver` 只有驱动，无体素/聚类/法线链路（TODO P2-3）。
3. **相机标定与深度-彩色对齐未做**（`docs/explanation/architecture.md` §7 已列为缺口）。
4. **臂的几何/限位是占位值** —— 抓取位姿的工作空间过滤在拿到真实参数前只能"跑通不报错"。
5. **超远物体建模的策略待定**：研究结论建议"Nav2 + 预建图先接近（纯定位模式），
   再局部稠密建模（RTAB-Map/OctoMap，~0.05 m），用完丢弃"，不常驻大场景稠密图。
6. **GraspNet 的具体部署**：需按本机 JetPack 7.2 重编 CUDA 算子（用 Jetson 专用 PyTorch wheel，
   不是 PyPI 通用版）；点数/裁剪参数要在 4 GB 下压测。
7. **`moveit_task_constructor` 未安装**，需加入环境。

---

## 参考

- GraspNet-1Billion：<https://github.com/graspnet/graspnet-baseline>（非商业许可）
- AnyGrasp SDK：<https://github.com/graspnet/anygrasp_sdk>（研究用途、机器指纹锁定）
- GPD：<https://github.com/atenpas/gpd>（BSD，CPU）
- `pcl_ros` Jazzy 状态：<https://index.ros.org/p/pcl_ros/>
- Jazzy blocked releases（`moveit_grasps`）：<https://repo.ros2.org/status_page/blocked_releases_jazzy.html>
- Jetson 内存优化：<https://www.jetson-ai-lab.com/tutorials/ram-optimization/>
- 本文结论由三路研究中的"抓取栈"专项得出（2026-10-05）
