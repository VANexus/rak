# HANDOFF — LFD-RoadSeg 复现 · Jetson 部署 · 室内微调（交接文档）

> 面向接手的 agent / 协作者。**读完本文即可继续工作，无需回溯对话**。
> 最后更新：2026-11-11。上一份工作日志见同目录 `DEBUG_LOG.md`、复现报告见 `REPRODUCTION_REPORT.md`。

---

## 0. 一句话现状

LFD-RoadSeg 论文复现已完成（MaxF 95.94 vs 论文 96.28），室内版已在 Jetson Orin Nano 实现 **30 FPS 实时道路分割推流**；室内微调将 IoU 从 **17.6% 提升到 78.8%**，三域版 v2 已训出（CamVid 90.9%、室内 68.8%）。2026-11-11 已在 `VANexus/rak` 本地工作区加入 ROS 2 感知包和 RViz/Nav2 接线；RTX 4080 上 TorchScript 推理优化为 **2.51 ms → 1.47 ms**。**当前车端验证被 Jetson 网络不可达阻塞**；ROS 包尚未在 Jetson 构建/运行，v2 也尚未上车验证（见 §6）。

---

## 1. 项目是什么

| 项 | 内容 |
|---|---|
| 任务 | 论文 *Exploiting Low-level Representations for Ultra-Fast Road Segmentation*（LFD-RoadSeg, IEEE TITS 2024, arXiv:2402.02430）的**复现 + 车载部署 + 场景微调** |
| 上游代码 | https://github.com/zhouhuan-hust/LFD-RoadSeg（公开） |
| 部署目标 | Jetson Orin Nano 10W + Orbbec ASTRA S 深度相机（倒装）+ ROS 2 Jazzy 小车（rak 仓库） |
| 用途 | 小车自主导航的视觉感知；Nav2 主通道仍为激光雷达 costmap，本模块为增强项 |

---

## 2. 环境（重要，别踩坑）

| 项 | 值 |
|---|---|
| 训练/开发机 | `D:\conda_envs\pytorch_d\python.exe`（Python 3.10, torch 2.12.0+cu126, RTX 4080 Laptop 12G） |
| Jetson | `ssh xrak`（= orin.xrak）/ `ssh xrak-lan`（= 192.168.5.215），密钥 `~/.ssh/rak`，密码免输 |
| Jetson 环境 | torch 2.12.0+cu132, cv2 4.11, numpy 1.26.4（**零安装，别再装包**） |
| **网络坑** | 本机 192.168.5.191。**代理软件（Clash/TUN）会把 22 端口劫持**，症状 `Connection closed by 198.18.1.1`——先关代理或用 `xrak-lan`。2026-11-11 Jetson 完全不可达（ping 50% 丢包、SSH 超时），需现场确认车是否开机/联网 |
| 工作区纪律 | `D:\Desktop\SNN` 外改动先问用户；`official_repos/` 只读；不 pip/conda 装包（缺东西写垫片） |

---

## 3. 关键结果（全部实测）

### 3.1 论文复现（KITTI Road，kittival 五折交叉验证，官方 devkit BEV 指标）

| 指标 | 论文 Table VIII | 本复现 | 偏差 |
|---|---|---|---|
| MaxF (%) | 96.28±0.14 | **95.94±0.26** | −0.34 |
| PRE / REC (%) | 96.56 / 96.00 | 96.31 / 95.58 | −0.25 / −0.42 |
| 参数量 | 936,067 | **936,067** | 逐位一致 |
| MACs | 8.392 G | **8.392 G** | 逐位一致 |

分折 MaxF：96.05 / 95.83 / 95.49 / 96.11 / 96.24。
差距主因：论文两步训练（320×500 裁剪→全图）的**第一步在开源代码中被注释**。

### 3.2 部署性能

| 项 | 数值 |
|---|---|
| RTX 4080（375×1240） | 296 FPS（3.37 ms） |
| Orin Nano 10W（375×1240） | 19.2 FPS（52.1 ms） |
| **Orin Nano 10W（187×620，论文 Jetson 口径）** | **65.6 FPS（15.25 ms）** — 超论文 TX2 的 54 FPS |
| 相机采集 | 29.6 FPS（30Hz 满速） |
| 端到端实时推流 | **29-30 FPS**（车上处理，电脑仅显示） |

### 3.3 模型对比（★部署选型依据）

| 模型 | 室内 IoU（**无效**，见 §3.4） | CamVid IoU | CamVid BF1 | 说明 |
|---|---|---|---|---|
| `exp1`（KITTI 原版） | 0.1763 | 0.8568 | — | 基线 |
| `indoor` v1 | 0.7876 | 0.4722 | — | **CamVid 灾难性遗忘** |
| `v2` 三域混合 | 0.6878 | 0.9088 | 0.5884 | 泛化好 |
| `v3` 边缘感知 | 0.8206† | **0.9370** | **0.6575** | ★室外/路口最优（边缘损失 BF1 +6.9pp）；†室内数字为倒图坐标系，**部署方向 0.0056 作废** |
| `v4` 正立修正 | 0.8088 | 0.9334 | 0.6346 | 修正方向，但室内标签仍是伪标签（§3.4） |
| **`v6` 手动多边形标签 + 边缘感知** | **0.9978（部署方向实测）** | 0.9383 | 0.6587 | ★**最终部署模型**——可信地面标签 + 双域无遗忘；替换 indoor.ts 上车 |

**可用结论（2026-11-13）**：室外/路口用 **`v3`**；**室内/固定场地用 `v6`（部署方向实测 IoU 0.9978）**——三域全绿，跷跷板问题解决。

### 3.4 ⚠️ 两代踩坑记录（室内结果作废史）
1. **伪标签陷阱**：`pseudo_label.py` 用 KITTI 模型自信输出做种子自训练——标签 99.6% 落在天花板区（地面区污染仅 0.4%），v1-v4 的室内 IoU 全是对错误标签的自证，作废
2. **方向不一致**：`burst_*.raw` 是相机原始倒图，v3 前训练用倒图而部署（feeder）用正立图——部署方向 IoU 实测 0.0056
3. **修复（v6）**：`label_manual/`（多边形正立标签）+ burst 翻正训练 → 部署方向 0.9978

### 3.4 ⚠️ 伪标签陷阱（2026-11-11 发现，室内结果全部作废）

`pseudo_label.py` 用 KITTI 模型自信输出做种子生成标签，再自训练 —— **该循环把模型错误固化成了"真值"**：

- 原始标签（`ft_data/label_*.png`）**99.6% 落在画面上部** = 天花板/墙壁/柜子（8 帧实测：地面区污染率仅 0.0-0.8%），地板几乎无标签
- 铁证：把标签限制在"正立图 `y>0.42H`"（地面几何先验）后，**覆盖率 = 0.0%**
- 因此 v1/v2/v3/v4 的"室内 IoU 0.68–0.82"只是**对错误标签的一致性**，不代表能识别地面
- 独立的另一个 bug：`burst_*.raw` 是**相机原始倒图**，而 feeder 部署时已垂直翻正 → v3 前训练与部署方向相反，实测部署方向 IoU **0.0056**（v4 已修正为 0.8114，但标签仍错）

**结论：伪标签自训练在本数据上彻底失败，不要再加训练轮次。**

### 3.5 地面感知的正确路径（按可靠性排序）

| 方案 | 成本 | 可靠性 | 说明 |
|---|---|---|---|
| **A. 部署端几何先验** | 10 分钟，零训练 | 中 | 相机固定 → 只保留正立图 `y>0.42H` 的输出并边缘平滑。立刻消灭"标天花板"，但无法区分地面与地面障碍物 |
| **B. 人工标注 10–20 帧** | 用户标 30–60 分钟 | **高** | 画地面多边形 → 转掩码 + 光流传播 → 微调（10 分钟）。唯一能产出可信室内模型的路径 |
| **C. 深度几何（需先修标定）** | 半天 | 中高 | 结构光对黑地毯无效（有效像素 15%），木地板/浅色地垫有效；需正确内参 + 深度-彩色配准后重做 RANSAC |
| D. 换相机（Gemini 335L 主动双目） | 硬件 | 高 | 对暗面鲁棒，根治方案 |

**推荐**：先上 A（今天就能改善可视效果），同时按 B 标 10 帧 —— 两者结合才有既可用又可信的方案。

---

## 4. 文件地图

### 4.1 本机 `D:\Desktop\SNN\LFD-RoadSeg-reproduce\`

```
LFD-RoadSeg/                    官方代码克隆（含 Windows/numpy2 兼容修复，见 REPRODUCTION_REPORT.md §3）
  outputs/exp1/checkpoints/model_epoch_150.pth   五折第 1 折训练权重（复现用）
data/
  kitti/{training,testing}      KITTI Road 原始数据（289 训练/290 测试）
  kittival/exp1..5/             五折划分（scripts/make_kittival.py 生成）
  kitti_mask/                   289 张 KITTI GT 转二值掩码（v2 训练用）
camvid_repo/CamVid/             CamVid 数据集（train 367 + test 233，road=class 3）
ft_data/                        室内自采 40 帧（burst_*.raw 彩图 + label_*.png 伪标签），共 80 文件
deploy_pkg/                     ★核心部署包（见 §4.2）
scripts/                        五折脚本 / depth_autolabel.py（深度自动标注）/ kitti_gt_convert.py
vis_samples/                    所有可视化证据（双机对比、暗区修复、before/after、视频）
REPRODUCTION_REPORT.md          复现报告（精度/速度/差异分析）
DEBUG_LOG.md                    2026-11-08 排障纪要（含 color 流根因、v2 bug 遗留说明）
HANDOFF.md                      本文件
```

### 4.2 `deploy_pkg/`（本地 ↔ 车上同步）

| 文件 | 作用 |
|---|---|
| `lfd_roadseg_exp1.ts` | KITTI 版 TorchScript（3.93MB） |
| `lfd_roadseg_indoor.ts` | **室内版 v1**（IoU 78.8%） |
| `lfd_roadseg_v2.ts` | **三域版 v2**（camvid 90.9%）★未上车 |
| `v2_best.pth` / `indoor_best.pth` | 对应 state_dict |
| `finetune_indoor.py` / `finetune_v2.py` | 微调脚本（v2 已能跑通，25ep 约 10 分钟） |
| `pseudo_label.py` | 伪标签（模型自信区种子 + Lab 区域生长） |
| `jetson_seg_stream.py` | **车上分割推流服务**（读共享内存→推理→JPEG 推 TCP:9001） |
| `seg_viewer_win.py` | Windows 显示端（连 9001，ESC 退出，自动重试连接） |
| `jetson_bench.py` / `record_video.py` / `burst_infer.py` | 一致性验证 / 录像 / 批量推理 |
| `dark_fix_test.py` / `before_after.py` | 暗区参数对比实验 |

### 4.3 车上 `/tmp/oni_old/`（重启即失，重建命令见 §7）

| 文件 | 作用 |
|---|---|
| `feeder`（C++） | 采集 640×480 RGB888 → 垂直翻转 → 写 `/dev/shm/cam_frame.bin`（原子 rename） |
| `jetson_seg_stream.py` | 同上 4.2，模型路径 `/tmp/lfd_deploy/lfd_roadseg_indoor.ts`，`RES=(480,640)` |
| `grab2` / `burst` / `data_collect` | 双机抓帧 / 连拍 / 彩深成对采集 |
| `Include/` + `libOpenNI2.so` | **OpenNI-Linux-Arm64-2.3 老栈**（color 流可用关键） |
| `data/` | 300 对历史采集数据（深度大多无效） |
| `*.cpp` | 上述工具源码 |

### 4.4 本机 `D:\Desktop\navigator\rak_win\tools\orbbec_dl\`

诊断套件：`oni_old/arm64/`（★可用老 OpenNI2 栈）、`oni2_linux/`、Win 驱动、官方 Dropbox 12 包、`ros_astra_camera/`。

---

## 5. 核心原理与关键决策

### 5.1 为什么必须用老 OpenNI2（color 流排障结论）

- ASTRA S 固件 `RD109Y-007`（2017）与 OrbbecSDK **v1.10.18** 协议不兼容：color 启动必报 `Start stream failed, OB_SENSOR_COLOR Match openni video mode failed!`（`switchD2CParams` 响应 id 不一致）
- **双机 A/B 证明非硬件**（SN 17111811310 / 17111811554 同故障）
- 换**同代 OpenNI-Linux-Arm64-2.3**（Dropbox 包 `pkg2_ou49febb83m476d`，含 `libPSLink.so`）后**双机彩流全通，无需刷固件**
- 深度→彩色**板载配准不支持**（`registration mode set failed`），靠自动标定水平偏移（实测 −8px）
- 其他：apt 的 `orbbec_camera`（SDK v2）**完全不认这台相机**；`rak_astra_driver`（SDK v1.10）能开深度但 color 必败

### 5.2 部署链路（当前形态）

```
ASTRA S ──OpenNI2老栈──> feeder(垂直翻转) ──/dev/shm/cam_frame.bin──> jetson_seg_stream.py
   (30Hz)                    [看门狗]              (seq 计数)          (自适应gamma→模型→th0.40→JPEG)
                                                                              │ TCP :9001
                                                              seg_viewer_win.py (Windows 显示)
```

- **相机倒装 = 垂直翻转**（不是旋转180°），修正在 feeder 源头单点完成
- 自适应 gamma：亮度 <110 才做 `gamma0.6` 提亮（暗区召回），亮场景直通（防过曝）
- 分割阈值 0.40（默认 0.5 偏保守）
- 双看门狗：`feeder` 与 `jetson_seg_stream.py` 各一个 `while true` 循环，死了 3 秒自动拉起

### 5.3 为什么室内需要微调

- KITTI 模型对室内地板 IoU 仅 **17.6%**（域差距 + 位置先验："路在画面中下方"）
- 深度自动标注**失败**：ASTRA S 是结构光，黑色地毯不反射红外 → 深度有效像素仅 15%，RANSAC 只能拟合到墙（法向 ny≈0）。**这是传感器原理限制，非代码问题**
- 改用**伪标签自训练**：模型自信区（th 0.3-0.5）做种子 + Lab 颜色区域生长扩展到两侧 → 40 帧标签 → 微调 30ep → **IoU 78.8%**
- 打破位置先验：v2 训练时 **随机裁剪**（p=0.6，480×640→360×480）+ 三域混合

---

## 6. 当前状态 & 待办（按优先级）

### 🔴 P0：Jetson 不可达（2026-11-11）

- 症状：`ssh xrak` → `Connection closed by 198.18.1.1`（代理劫持）；`xrak-lan` → timeout；`ping 192.168.5.215` 50% 丢包
- 需现场确认：车是否开机、是否连同一 Wi-Fi、IP 是否变了（登录路由器看）
- 排查顺序：关代理软件 → `arp -a` 看能否解析 MAC → ping 网关 → 检查车联网

### 🟠 P1：v2 上车验证（未做）

```bash
scp deploy_pkg/lfd_roadseg_v2.ts xrak:/tmp/lfd_deploy/
# 车上改 jetson_seg_stream.py 两处：
#   ts = torch.jit.load("/tmp/lfd_deploy/lfd_roadseg_v2.ts", map_location="cuda")
#   RES = (480, 640)
ssh xrak 'pkill -9 -f "jetson_seg_stream.p[y]"'   # 看门狗 3 秒自动拉起新配置
# 本机: python deploy_pkg/seg_viewer_win.py        # 看效果
```

### 🟡 P2：v3 微调（解决 v1↔v2 的跷跷板）

思路：以 `v2_best.pth` 为起点、提高室内数据权重（如 indoor×8）再训 —— 目标 indoor ≥75% 且 camvid ≥88%。
或：场地切换机制（室内用 v1 权重、室外用 v2）。

### 🟢 P3：ROS 视觉导航接线（软件骨架已写，车端待验证）

模块归属为机器人 ROS 仓库 `D:\Desktop\rak`（Git remote `https://github.com/VANexus/rak`），包路径 `src/rak_road_perception`；LFD-RoadSeg 训练/权重仍由本工作区管理，模型文件不放入 RAK 源码，车端默认从 `/tmp/lfd_deploy/lfd_roadseg_indoor.ts` 加载。

节点支持 Astra 共享内存帧或 ROS `Image` 输入，发布 `/road_perception/road_mask`（mono8）和 `/road_perception/overlay`（bgr8）供 RViz 查看；`task_navigation.launch.py` 有 `road_perception:=true`、`road_rviz:=true` 开关。Nav2 局部 costmap 增加独立语义 `ObstacleLayer`：高置信道路点仅 clearing、高置信非道路点仅 marking，使用 max 合并保留 LiDAR 障碍；视觉点云默认关闭，全局 costmap、任务目标和速度控制链不由该模型接管。Astra 彩色输入沿用 OpenNI2 feeder 写入共享内存，没有改相机驱动。接入说明及启动命令见 `D:\Desktop\rak\docs\road_perception.md`，架构决策见 `D:\Desktop\rak\docs\adr\0019-roadseg-navigation-input.md`。

本机 RTX 4080、640×480 室内帧上比较 TorchScript 基线与 `torch.jit.freeze` + `torch.jit.optimize_for_inference`：推理从 **2.51 ms 降至 1.47 ms**，阈值掩码 IoU **0.99983**；FP16 和 channels-last 在该机测试更慢，故默认保留 FP32。此数据是开发机模型推理对比，不是 Jetson 端到端结果。

本次已获用户授权并按 `rak/AGENTS.md` 实施；**尚未在 Jetson 构建、启动或完成车端测试**（Jetson 网络不可达）。相机到 `base_footprint` 外参与 RGB-D 配准未实测，所以 `road_publish_nav_clouds` 默认关闭。车端恢复后先构建并检查模型、图像方向、RViz 和 TF，再逐步打开语义点云并观察局部 costmap；低速导航验收另按硬件门槛执行。当前 `task_navigation.launch.py` 默认 `motion_backend:=sim`，只验证软件链路，不驱动实车底盘。

### 🟢 P4：其他

- 补充正规 KITTI 两步训练（追平 96.28）
- `astra_bridge_node` ROS 化后，把 TCP 推流降级为调试手段
- 车上 VIA hub USB 供电不稳（两相机同挂必掉线）→ 换带供电 hub 或分插两个 root 口

---

## 7. 常用命令速查

### 7.1 车上全链路重建（重启后 /tmp 清空时）

```bash
# 1) 部署老 OpenNI2 栈（本机→车上）
scp -r "D:/Desktop/navigator/rak_win/tools/orbbec_dl/oni_old/arm64/OpenNI-Linux-Arm64-2.3/Samples/Bin/." xrak:/tmp/oni_old/
scp -r "D:/Desktop/navigator/rak_win/tools/orbbec_dl/oni_old/arm64/OpenNI-Linux-Arm64-2.3/Include" xrak:/tmp/oni_old/
scp "D:/Desktop/SNN/LFD-RoadSeg-reproduce/deploy_pkg/lfd_roadseg_indoor.ts" xrak:/tmp/lfd_deploy/
scp "D:/Desktop/SNN/LFD-RoadSeg-reproduce/deploy_pkg/jetson_seg_stream.py" xrak:/tmp/oni_old/

# 2) 编译 feeder（源码在车上 /tmp/oni_old/feeder.cpp，若丢了从本机 _tmp_feeder.cpp 恢复）
ssh xrak 'cd /tmp/oni_old && g++ -O1 -I Include feeder.cpp -L . -lOpenNI2 -Wl,-rpath,/tmp/oni_old -o feeder'

# 3) 双看门狗启动（注意 cd，否则 python 找不到脚本 —— 踩过这个坑）
ssh xrak 'cd /tmp/oni_old && setsid nohup bash -c "cd /tmp/oni_old && while true; do ./feeder >> /tmp/feeder.log 2>&1; sleep 2; done" >/dev/null 2>&1 </dev/null &'
ssh xrak 'cd /tmp/oni_old && setsid nohup bash -c "cd /tmp/oni_old && while true; do python3 jetson_seg_stream.py >> /tmp/seg_stream.log 2>&1; sleep 3; done" >/dev/null 2>&1 </dev/null &'

# 4) 本机看画面
D:/conda_envs/pytorch_d/python.exe "D:/Desktop/SNN/LFD-RoadSeg-reproduce/deploy_pkg/seg_viewer_win.py"
```

### 7.2 状态检查（一次取证）

```bash
ssh xrak 'pgrep -c feeder; pgrep -c -f "jetson_seg_stream.p[y]"; tail -3 /tmp/seg_stream.log; ss -tn | grep 9001; cat /dev/shm/cam_seq'
```

### 7.3 重跑复现 / 微调

```bash
# 五折复现（约 1.8h）
D:/conda_envs/pytorch_d/python.exe scripts/run_all_folds.py 1 2 3 4 5
# 室内微调 v1（约 30s-1min）
D:/conda_envs/pytorch_d/python.exe deploy_pkg/finetune_indoor.py
# 三域微调 v2（约 10min）
D:/conda_envs/pytorch_d/python.exe deploy_pkg/finetune_v2.py
```

---

## 8. 踩过的坑（别重复）

| 坑 | 症状 | 解法 |
|---|---|---|
| 代理劫持 SSH | `Connection closed by 198.18.1.1` | 关代理，或用 `xrak-lan` 直连 IP |
| `pkill -f "astra_node"` 自杀 | ssh 会话直接断 | 模式写成 `"astra_nod[e]"`（括号打断自匹配） |
| 看门狗 cwd | 日志刷 `can't open file...jeton_seg_stream.py` | `bash -c` 里必须 `cd /tmp/oni_old` |
| OOM 静默杀进程 | 服务无声消失、无 traceback | Orin 仅 3.4G 统一内存，别并行跑多路 CUDA；用看门狗自动拉起 |
| 死客户端卡死推流 | 进程活着、不监听、`ss` 无连接 | 服务端 `cli.settimeout(30)` + 捕获 `socket.timeout` |
| 相机倒装方向搞错 | 先旋转180°发现镜像 | ASTRA S 倒装是**垂直翻转**，改 `flipv_rgb888` |
| OpenNI2 C++ 链接 | `undefined reference to ob::Context::Context` | `-lOrbbecSDK` 必须放在源文件**之后**；C++ 包装还有 ABI 问题 → 用纯 C API |
| OpenNI2 头文件 | `OpenNI.h: No such file` | 用 `-I Include`（`Include/` 下直接放 `OpenNI.h`） |
| 深度自动标注全灭 | `LABELED: 0 masks kept` | ①采样限倒装原始图**上部 65%**（地面在那里）②约束法向 `|ny|>0.6` ③掩码需 `flipud` 转正 |
| numpy 负步长 | `negative strides are not supported` | 翻转增强后加 `.copy()` |
| `CosineAnnealingLR(T_MAX=)` | `unexpected keyword` | 参数名是 `T_max` |
| numpy 等值判断 | `truth value of an array is ambiguous` | 列表去重用索引集合，不要 `x in list_of_arrays` |
| LFD-RoadSeg forward 入参 | `invalid data type 'str'` / dict 错误 | 模型签名是 `model({"img": x})`，不是 `model(x)` |
| Windows 幽灵设备 | PnP 里有已拔掉的 2bc5 设备 | `Get-PnpDevice -PresentOnly` |

---

## 9. 与其它工作区的关系

- **HAR 主线**（`D:\Desktop\SNN\har` 等）与本任务**无关**，别混
- **Jetson 小车仓库**：`D:\Desktop\rak`（GitHub `VANexus/rak`，ROS 2 Jazzy 分层）；`D:\Desktop\navigator\vehical_rak_chassis` / `vehical_rak_edge_ai`（STM32 固件，只推车上裸仓库 `xrak:`）
- **改任何上述仓库前必读其 `AGENTS.md`**（用户明确要求：严格遵守规范，不做无规则修改）
- 阿克曼机器人（ROSmini）资料：`D:\Document\deepseek-harness\default-workspace\ackermann_extract`

---

## 10. 上手指南（新的 agent 第一小时做什么）

1. 读本文件 §0-§5（10 分钟）
2. 按 §7.2 检查 Jetson 是否可达；不可达 → 报给用户（§6 P0）
3. 可达 → 按 §7.1 起链路，本机开 viewer 看实时画面
4. 确认基线：当前车上跑的是 `indoor v1`；若要看泛化更好 → 按 §6 P1 换 v2
5. 之后按 §6 的 P2/P3/P4 推进
