# 点云抓取：离线 GraspNet 与隔离仿真

## 范围与开发位置

第一阶段：Gemini 335L 真实 RGB-D → 本机 RTX 4060 离线 GraspNet → RViz 选候选 → MoveIt **规划预览**。
代码在 `rak`，开发分支 `feature/grasp-perception`，基线主线 `89fe7feb5f43884ada9f93a4db7890a024d7c772`。
Windows 工作树 `D:\car\rak-grasp`；Orin 验证工作树 `/home/xrak/rak-grasp-source`。
原 `D:\car\rak` 和 `/home/xrak/rak` 不用于覆盖开发；现有修改保留。

这按用户批准的阶段顺序，先做 GraspNet 离线开发；ADR-0005 的几何优先路线未被改写。
不接语音、导航、DeepStream、Holoscan、真实运动，不改变决策数据类型或 Orin 推理环境。

## 组件和环境

| 位置 | 用途 |
| --- | --- |
| `rak_bringup` | 复用 Gemini 驱动（`grasp_camera.launch.py`）和序列号选择 |
| `rak_pointcloud` | 可选实时 GPU 点云预览；离线数据自行按保存的完整标定重建 |
| `rak_grasp_perception` | 有界快照、离线推理、去重、碰撞过滤、ROI、RViz、规划请求 |
| `rak_interfaces` | `GraspCandidate`、`GraspCandidates`、`SelectGrasp`，只增接口 |
| `rak_moveit_config` | `grasp_sim.launch.py`，复用既有六轴软件夹具 |
| `scripts/graspnet` | 固定版本清单、开发镜像、兼容补丁、官方样例转换 |
| `D:\car\tools\graspnet` | 第三方源码、模型、缓存、镜像元数据、样本和结果 |

Orin 实测系统 Ubuntu 24.04 / ROS 2 **Jazzy**。本机目标同样是 WSL2 Ubuntu 24.04 + Jazzy。
推理容器 `nvcr.io/nvidia/pytorch:24.05-py3` 内部为 Ubuntu 22.04 / Python 3.10 / CUDA 12.4.1；
它只运行一次性离线工具，通过文件传递结果，ROS 节点继续使用 Ubuntu 系统 Python。
本机 RTX 4060 Laptop 8188 MiB，驱动 560.94；不在 Orin 安装 PyTorch，不添加在线模型服务。

来源：[NVIDIA 24.05 说明](https://docs.nvidia.com/deeplearning/frameworks/pytorch-release-notes/rel-24-05.html)、
[GraspNet 官方仓库](https://github.com/graspnet/graspnet-baseline)、
[ROS Jazzy 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。

### 本机环境安装与重启

2026-10-08 最初自动审批拒绝安装。用户随后明确批准并要求 D 盘安装，已完成虚拟机平台与 WSL 功能启用，
WSL 3.0.1 MSI 安装返回 0，实际安装目录 `D:\car\tools\graspnet\wsl-runtime`。
安装器报告需要重启；用户已完成重启。
Windows 功能与安装器系统缓存仍由 Windows 管理；WSL 运行库、下载与模型位于 D 盘。
随后发现 D 盘仅余约 15.6 GiB，用户允许其他组件放 `E:\a_robot`。
Ubuntu 已注册为 `Ubuntu-24.04`，虚拟磁盘位于 `E:\a_robot\tools\graspnet\wsl\Ubuntu-24.04`。
实测 Ubuntu 24.04.5、WSL2 内核 6.18.40.1，`nvidia-smi` 已识别 RTX 4060 Laptop / 8188 MiB。
签名有效的 Microsoft MSI、安装日志和 `wsl-install-status.json` 均在 tools/graspnet。

新机器安装命令（本机已执行，参见 [Microsoft WSL 文档](https://learn.microsoft.com/en-us/windows/wsl/install)）：

```powershell
& D:\car\tools\graspnet\wsl-runtime\wsl.exe --install --from-file D:\car\tools\graspnet\ubuntu-24.04.5-wsl-amd64.wsl --name Ubuntu-24.04 --location E:\a_robot\tools\graspnet\wsl\Ubuntu-24.04
```

Docker 默认 `/var/lib/docker` 和 containerd 的 `/var/lib/containerd` 位于该 E 盘 ext4 虚拟磁盘中；
不把 overlayfs 放在 `/mnt/e` 的 DrvFS 上。安装后先验证：

```powershell
wsl --version
wsl --list --verbose
wsl -d Ubuntu-24.04 -- nvidia-smi
```

Ubuntu 内按 ROS 官方文档安装 Jazzy desktop、colcon、rosdep；按 Docker/NVIDIA 官方说明配置 GPU 容器。
可复跑本仓库仅限 x86 WSL2 的引导脚本：

```powershell
& D:\car\tools\graspnet\wsl-runtime\wsl.exe -d Ubuntu-24.04 -u root -- env ROS_APT_MIRROR=https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu bash /mnt/d/car/rak-grasp/scripts/graspnet/bootstrap_wsl.sh
```

该脚本不安装 Linux NVIDIA 显卡驱动，不创建项目 Python venv。安装日志在 `tools/graspnet/wsl-bootstrap.log`。
本机 ROS 主站下载缓慢，使用 [清华 ROS 2 镜像](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)，
保留 `ros2-apt-source` 提供的官方签名密钥。省略 `ROS_APT_MIRROR` 可使用官方默认源。
WSL/VPN 路径存在 IPv6 DNS 超时，脚本仅在此 Ubuntu 内为 apt 启用 IPv4 与下载超时。
基础镜像的 NVIDIA 附加 pip 索引在本机无法解析；Dockerfile 安装普通依赖时用
`PIP_CONFIG_FILE=/dev/null` 和 PyPI 官方源，避免逐包重试，保留镜像自带 PyTorch。
该变量的行为见 [pip 配置文档](https://pip.pypa.io/en/stable/topics/configuration/#pip-config-file)。
验证 `docker run --rm --gpus all nvcr.io/nvidia/pytorch:24.05-py3 nvidia-smi` 后才开始编译扩展。
24.05 基础容器内 GPU、PointNet2/KNN CUDA 扩展、checkpoint 加载和官方样例 FP32 推理均已通过；
具体耗时、显存、候选数与验证边界见 [grasp_validation.md](grasp_validation.md)。

### 固定第三方版本与模型

`scripts/graspnet/assets.json` 记录源码提交与用户提供权重的 SHA-256。
当前权重 `checkpoint-rs.tar` 12,468,415 字节，SHA-256：
`60680087c61cba2b6791614fef1519071e294f6dcaf99b3f581bb95f7c51a868`。
ZIP CRC、开发容器内参数加载和实际前向推理均通过。仅加载可信来源的 PyTorch checkpoint。
Google Drive 曾返回配额 HTML，已另存 `models/google-drive-quota.html`，不是模型。

容器补丁只修改镜像内的源码副本：Tensor `.data_ptr`/`.is_cuda`、无 GPU 构建时按 CUDA_HOME 编译 KNN、
精简 API 导入。自行实现有平行指爪 180°对称性的 SE(3) NMS，不引入额外旧 `grasp_nms` 扩展。
扩展编译和官方样例实测已通过；这不等于与上游旧环境完成数值一致性比较。
镜像同时清理基础 OpenCV 4.7 残留的 Python/native 配置后安装 headless 4.10，避免混合绑定错误。

在 WSL 仓库根目录：

```bash
bash scripts/graspnet/build_image.sh /mnt/d/car/tools/graspnet
python3 scripts/graspnet/prepare_example.py \
  --tools /mnt/d/car/tools/graspnet \
  --output /mnt/d/car/tools/graspnet/samples/official-demo-new
bash scripts/graspnet/infer.sh \
  /mnt/d/car/tools/graspnet/samples/official-demo-new \
  /mnt/d/car/tools/graspnet/samples/official-demo-new/roi.json \
  /mnt/d/car/tools/graspnet/samples/official-demo-new/grasps.json
```

镜像构建会核对源码提交和权重，记录 `image-inspect.json` 和 `container-python-freeze.txt`。
所有输出用新路径，避免覆盖样本。
若普通镜像拉取受到单连接限速，可使用仓库的有界分段下载入口（Windows 系统 Python，无需 PyTorch）：

```powershell
python D:\car\rak-grasp\scripts\graspnet\fetch_image.py --output E:\a_robot\tools\graspnet\ngc --workers 32
& D:\car\tools\graspnet\wsl-runtime\wsl.exe -d Ubuntu-24.04 -u root -- docker load --platform=linux/amd64 -i /mnt/e/a_robot/tools/graspnet/ngc/pytorch-24.05.oci.tar
```

仅从 NVIDIA 官方 registry 下载，固定多架构清单 digest `d4030fc…03adea`，只保存 linux/amd64 的层。
每层 SHA-256 校验后才创建 OCI 归档，临时访问令牌自动续期；缓存与归档放 E 盘。
导入后仍运行 `build_image.sh` 编译兼容扩展。分段下载过程不代表容器或模型已通过验证。
默认 FP32 / batch=1 / 20000 点 / seed=0。结果保存推理耗时、推理加过滤耗时、总处理耗时、
峰值 CUDA **allocated / reserved** 显存、点数、候选数和过滤计数。
`inference_seconds` 包含首次前向运行及解码，不含模型加载；`total_processing_seconds` 还包含
标定点云重建、模型加载和后处理，不含容器启动与最终文件写入。显存指标不包括驱动上下文，不作实时承诺。

## 构建

在 Jazzy 上，仅构建本阶段包（Orin 已有传感器运行工作区作为 underlay）：

```bash
source /opt/ros/jazzy/setup.bash
# Orin 验证时使用已有传感器组件；本机首次应先准备对应依赖。
source /home/xrak/rak/install/architecture/local_setup.bash
rosdep install --from-paths src/rak_grasp_perception --ignore-src -r -y
colcon build --base-paths src --executor sequential \
  --packages-select rak_interfaces rak_description rak_arm_simulation rak_grasp_perception rak_moveit_config \
  --build-base build/grasp --install-base install/grasp
source install/grasp/local_setup.bash
```

`rak_pointcloud` 的实时 GPU 节点依赖 JetPack，WSL 不需构建该节点来回放离线快照。
ROS 包依赖用 apt/rosdep；不建项目 Python venv。仓库全量构建的已知依赖限制见 `AGENTS.md`。

本机 WSL 使用独立入口，限定离线五个包；不遍历 JetPack 专用包。输出位于 Ubuntu ext4 的
`~/rak-grasp-workspace`（其 VHD 在 E 盘），源码仍在 D 盘：

```bash
bash /mnt/d/car/rak-grasp/scripts/graspnet/build_wsl.sh
source ~/rak-grasp-workspace/install/local_setup.bash
```

入口会执行抓取几何/接口测试、描述和 MoveIt 回归，以及决策层两项回归。

## Orin 真实快照

确保没有其他进程占用相机。两个终端都 source Jazzy、现有运行 overlay、新 grasp overlay。
调试使用域 77，避免接入运行域 47；大图像使用已验证的 CycloneDDS loopback 配置，
接收缓冲不足会导致图像丢包（本次临时 `net.core.rmem_max=8388608`，未写持久 sysctl）。

```bash
export ROS_DOMAIN_ID=77
ros2 launch rak_bringup grasp_camera.launch.py live_cloud:=false
# 另一个终端，使用相同 DDS 环境：
ros2 run rak_grasp_perception capture_snapshot \
  --output /home/xrak/tools/graspnet/samples/bottle-01 --seconds 30
```

抓取专用 YAML 启用 RGB，保持两路 1280×800 / 30 fps、未做深度注册；不更改日常导航相机默认配置。
快照包含 `color.png`、`depth-mm.png`、`snapshot.json`：两路 K/D、原始时间戳、光学帧、工厂内部 TF。
时间差 **>20 ms 拒绝**；无有效深度、缺少标定、TF 或格式不符则失败，30 秒有界等待。
工厂 depth→color 内部标定不是相机→机器人外参。

将整个快照目录复制到本机 tools 下。深度按自身内参和畸变校正生成米制点云，
通过内部 TF 和 RGB 内参投影上色、遮挡检查；不会把两路同坐标像素当作同一点。
离线完整点云保留桌面和邻近杂物用于碰撞检查；实时 `rak_pointcloud` 只作可选预览。

## RViz 选 ROI、推理与回放

在装有 Jazzy 的显示端 source 新 overlay：

```bash
export ROS_DOMAIN_ID=77
ros2 launch rak_grasp_perception replay.launch.py \
  snapshot:=/absolute/snapshot roi_output:=/absolute/roi.json
rviz2 -d $(ros2 pkg prefix rak_grasp_perception)/share/rak_grasp_perception/config/grasp_scene.rviz
```

选择 RViz **Interact** 工具拖动绿色 ROI 三轴；右键菜单缩放区域和 **Save target ROI**。
ROI 以相机深度光学帧、米为单位；让框包含整个物体，不能只选可见的几个点。
ROI 中的点会被仿真当作待夹取物体。选择时应排除桌面和邻近物体；桌面点留在 ROI 外，
才能作为固定障碍参与检查。当前没有自动物体分割、不可见背面重建或真实接触力估计。
第一次无结果时只显示点云和 ROI，这是正常状态。保存 ROI 后退出回放、运行上面的 `infer.sh`。
再次启动回放，增加 `result:=/absolute/grasps.json`，最多显示去重后的 50 个彩色 U 形夹爪。
点击紫色候选按钮或用服务选择：

```bash
ros2 service call /grasp/select rak_interfaces/srv/SelectGrasp '{id: 0}'
ros2 topic echo /grasp/status
```

状态给出候选 ID、位置、四元数 xyzw、开口米数、评分。过滤统计保存在结果 JSON 的
`metrics.filter_counts`（尺寸/ROI、场景碰撞）；规划拒绝原因见 `/grasp/sim_status`。
RViz 三维文字使用 ASCII 和下划线，避免 Ogre 默认字体的空格宽度与 UTF-8 显示问题；
始终显示 `Force_unmeasured`，ROI 右键菜单、`/grasp/status` 和日志提供“夹持力未测量”。
颜色只区分 ID。**评分不代表成功概率或夹持力；夹持力未测量。**

主要接口：`/grasp/scene` PointCloud2、`/grasp/candidates` GraspCandidates、`/grasp/markers` MarkerArray、
`/grasp/selected` GraspCandidate、`/grasp_interactive/update`。选中消息不锁存，启动规划器后需重新点选。
更改 ROI 后需要重新推理，旧候选不会自动被视作新 ROI 的结果。
完整通过的五段预览发布到 `/grasp/display_planned_path`，RViz 使用此话题；MoveIt 默认
`/display_planned_path` 中可能只有局部预抓取轨迹，不作为抓取通过证据。拒绝结果不会发布新的完整预览。

## 隔离的 MoveIt 规划预览

```bash
export ROS_DOMAIN_ID=77
ros2 launch rak_moveit_config grasp_sim.launch.py \
  snapshot:=/absolute/snapshot roi:=/absolute/roi.json \
  simulation_mount:=/absolute/virtual_mount.yaml rviz:=true
# 另一个终端启动带 result 参数的 replay，再点击候选。
```

`virtual_mount.yaml` 必须 `simulation_only: true`，源帧匹配快照；示例安装矩阵是**虚构的展示参数**，
需要按场景把目标放入虚拟机械臂工作区，不能用于实机。
默认起点使用软件夹具的七关节状态。挂载 YAML 可选 `fixture_start_positions`，按七个关节全名给出
虚构预览起点（六轴 rad、夹爪 m）；必须有限且满足既有软件夹具限位，并通过场景有效性检查。
此选项只填写规划请求和 `DisplayTrajectory.trajectory_start`，不修改关节反馈或发送运动指令。
候选遵循 GraspNet +X 接近、+Y 闭合；虚拟 TCP 在 tool0 +X 方向 0.08 m。
模拟夹具开口限 0–0.05 m；模型候选可达 0.10 m，超出模拟夹具范围会明确拒绝。
预抓取先求解碰撞检查开启的 IK，再使用明确的六轴关节目标规划，避免宽姿态容差导致后续直线段跳变。
若原始预抓取、接近或闭合被拒绝，尝试沿夹爪 +X 轴翻转 180° 的等价平行指爪姿态，保持位置、接近方向与开口；
两次均开启碰撞检查，状态记录最终采用的 `equivalent_roll`。

预览五段：打开夹爪 → 预抓取 → 接近 → 闭合 → 抬起 8 cm。
桌面和周围点云转成 2 cm 占用盒，保留近基座 0.9 m 范围内的点，目标和障碍各最多 4000 个碰撞盒，超过限制则拒绝。
体素在深度光学帧中生成，在 ROI 边界裁剪并分割，再转换到基座；混合体素的障碍部分仍然保留。
每面最多内缩 1 微米以分离浮点共面边界，这只是数值容差，不是经过测量的物理安全间隙。
这样避免独立体素化给目标与障碍生成同一整块碰撞盒。点云只表达可见表面，ROI 仍需操作者正确区分物体与桌面。
目标只允许与 tool0/虚拟夹爪接触；闭合检查状态有效性，抬起时挂载目标碰撞体，并再次检查挂载状态，
若冲突则报告碰撞对后拒绝。不会为使测试通过而关闭障碍碰撞。
笛卡尔路径要求近完整覆盖且通过碰撞/跳变检查。RViz 动画是规划展示，时间戳用于播放，
不等价于经过实机动力学验证的执行轨迹，也没有模拟软物体接触力。
规划器只调用规划、FK、场景和有效性服务，不调用 ExecuteTrajectory 或硬件驱动。

实机另需相机外参、TCP、指爪几何、开口映射、反馈和控制接口；夹持力需传感器或经过标定的反馈控制。

## 验收与复测

```bash
ROS_DOMAIN_ID=77 python3 -m pytest src/rak_grasp_perception/test -q
colcon test --build-base build/grasp --install-base install/grasp \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/grasp --verbose
PYTHONPATH=src/rak_agent python3 src/rak_agent/test/test_ai.py
# 实际模型结果的回放、选择服务和截图验收（已 source grasp overlay）
ROS_DOMAIN_ID=77 xvfb-run -a -s '-screen 0 1280x800x24' \
  python3 scripts/graspnet/verify_replay.py \
  --snapshot /absolute/snapshot --result /absolute/grasps.json \
  --output /absolute/new-replay-evidence
# 增加实际模型候选的五段规划验证，挂载文件必须明确为虚构仿真参数：
ROS_DOMAIN_ID=77 xvfb-run -a -s '-screen 0 1280x900x24' \
  python3 scripts/graspnet/verify_replay.py \
  --snapshot /absolute/snapshot --result /absolute/grasps.json \
  --candidate-id 0 --roi /absolute/roi.json \
  --simulation-mount /absolute/virtual_mount.yaml \
  --output /absolute/new-model-simulation-evidence
```

验收记录见 [grasp_validation.md](grasp_validation.md)。官方样例 + 不透明瓶子、纸盒各三种姿态需要逐个保存
快照、ROI、结果、指标和截图。操作者实际重新摆放物体后采集，不能把同一场景复制六份作为验收。
应覆盖可达、不可达、障碍碰撞、空点云、缺失标定、不同步。本机 CUDA 构建和官方场景推理已通过，
现场瓶子/纸盒六组摆放仍待采集；不能报告实机抓取或六场景成功。
