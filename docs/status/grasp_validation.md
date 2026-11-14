# 点云抓取验证记录

> 状态：现行 · 最后更新：2026-10-14

日期：2026-10-08 开始，2026-10-09 继续。开发基线 `89fe7fe`，分支 `feature/grasp-perception`。
此文件区分已经实测的软件能力和等待环境、实物的验收项。

## 已验证

| 项目 | 结果与证据 |
| --- | --- |
| 模型文件 | 用户提供 RealSense checkpoint，12,468,415 字节；SHA-256 见 assets.json，ZIP CRC、网络权重加载和 RTX 4060 FP32 前向运行通过 |
| 官方样例 | 官方 RGB、深度、内参、workspace mask 已转换到样本契约；`official-demo/grasps-01.json` 生成 50 个候选并保存完整场景 |
| ROS 构建 | Orin Jazzy 上 `rak_interfaces/rak_description/rak_arm_simulation/rak_grasp_perception/rak_moveit_config` 五包通过 |
| 本机 ROS 构建 | WSL2 Jazzy 五包通过；`ros-build-08.log`：新增 FP32 边界与 ROI 混合体素回归后 14 项 pytest、5 项 CTest、`test_ai.py` 与 `test_task_intent.py` 全部通过 |
| 描述/MoveIt 回归 | 5 项 CTest 通过；安装仓库已声明的 `python3-defusedxml` 后复测通过 |
| 决策层 | `PYTHONPATH=brain python3 brain/tests/test_ai.py` 通过，冻结类型未改 |
| 几何与接口 | Orin 11 项 pytest 全通过；涵盖几何单位、标定投影、空云、缺失标定、不同步、ROI、去重、候选元数据、ROS 选择和 stride；Windows 9 项通过，ROS 模块跳过 |
| 真实同步快照 | Gemini 335L，1280×800，RGB/depth 时间差 1.111 ms；833,928 点，有效深度占 81.438%；中位深度 2.239 m |
| RViz 真实点云 | Orin Xvfb + Mesa 软件渲染；真实颜色点云与 ROI 回放启动通过；不启用实体桌面 |
| RViz 轨迹图形 | `sim-smoke-rviz-03`：合成候选与机械臂预览实际截图可见，Global Status OK，RViz 日志无 ERROR；已修复子 launch 参数污染与缺失 SRDF |
| 五段规划 | 合成软件夹具通过 open / pregrasp / approach / close / lift，发布 DisplayTrajectory 五段 |
| 本机 RViz / 选择服务 | `wsl-sim-smoke-03`：通过 `/grasp/select` 选中候选并完成五段规划，截图显示位置/姿态/开口/评分、Force_unmeasured 与机械臂；RViz 无 ERROR。修复 Ogre 文字间距及宽姿态目标带来的偶发 Cartesian 拒绝 |
| 实际模型候选回放 | `official-replay-01`：50 个官方样例候选、206290 个显示点；`bottle-replay-02`：4 个瓶子候选，位置/姿态/开口/评分与过滤计数可见，RViz 无 ERROR，截图已检查 |
| 碰撞盒划分 | ROI 边界混合体素切分后目标与障碍不再生成重叠整盒，保留六侧障碍与所有观测点（1 微米数值容差）；空集合与数量上限回归通过 |
| 最终本机回归 | `ros-build-12.log`：五包构建、14 项 pytest、5 项 CTest 与两项决策回归通过；`wsl-sim-smoke-07`：新完整预览话题的五段轨迹/RViz 通过，不可达和碰撞拒绝后没有发布新预览，截图已检查 |
| 不可达 | 合成目标超出机械臂范围，拒绝 `pregrasp IK/planning rejected: 99999` |
| 碰撞 | 合成阻挡物导致开爪起始状态碰撞，明确报告与 forearm/gripper/tool0 等连杆接触并拒绝 |
| 运动隔离 | 域 77，只用规划/FK/场景服务，不调用执行动作；未连接硬件运动接口 |
| WSL 安装 | WSL 3.0.1 位于 D 盘 tools/graspnet/wsl-runtime；用户重启完成，WSL2 内核 6.18.40.1 正常 |
| Ubuntu / GPU | 官方镜像 SHA-256 已核对；Ubuntu 24.04.5 已安装到 E 盘独立 VHD，WSL 内识别 RTX 4060 Laptop / 8188 MiB / Windows 驱动 560.94 |
| 容器运行库 | Docker Engine 29.8.2 / NVIDIA Container Toolkit 1.20.1；24.05-py3 容器内 `nvidia-smi` 实测识别 RTX 4060 / 8188 MiB / 驱动 560.94 |
| NVIDIA 基础镜像 | 24.05-py3 的固定 manifest 和 amd64 分层已全部下载并通过 SHA-256 校验；OCI 归档位于 E 盘，Docker 导入后的 RepoDigest 与固定 digest 相同；重复 blob 缓存已清理 |
| CUDA 开发镜像 | PointNet2、KNN、GraspNet API 编译及实际 CUDA 扩展导入通过；PyTorch `2.4.0a0+07cecf4168.nv24.05` / CUDA 12.4；镜像 ID `sha256:54529afd84f033bcd148492fb5e28a98abbc6254d353c7fa9b2e3df9a321e43a` |

Orin 证据：`/home/xrak/rak/tools/graspnet/samples/live-20261008`、`sim-smoke-05/results.json`、
`sim-smoke-05/launch.log`、`rviz-real-cloud.png`。Windows 真实快照与截图在 `D:\car\tools\graspnet`。
合成轨迹截图：`D:\car\tools\graspnet\rviz-synthetic-trajectory.png`；明确不是 GraspNet 实际推理结果。
真实 RGB 图目前是会议桌和墙面，未放置瓶子或纸盒，不算物体抓取样例。
操作员确认实验室已关灯，要求先推进其他工作；瓶子和纸盒六组实拍验收延后。
仿真测试使用的点云和候选明确标为 `synthetic integration fixture`，不是模型识别结果。
MoveIt 上游进程在部分测试关闭时出现过退出信号/析构报错；运行期间规划测试通过，所有本次进程已清理。

## 环境验证剩余项

1. 重启、Ubuntu 安装和 GPU 可见性检查已完成。
2. Ubuntu 位于 `E:\a_robot\tools\graspnet\wsl\Ubuntu-24.04`。源码、模型与 WSL 运行库保留 D 盘。
3. Jazzy、Docker Engine 与 NVIDIA Container Toolkit 已安装。Docker 默认 ext4 数据目录位于该 E 盘 VHD 内，避免把 overlayfs 放在 DrvFS 目录上。
4. `rak-graspnet:24.05` 已构建；PointNet2/KNN CUDA 扩展和官方 checkpoint 加载通过，基础 digest、派生镜像 ID 和完整 pip 清单已记录。
5. 官方样例 GPU 推理、候选回放和选择服务通过。完整五段轨迹由合成夹具验证通过；实际模型候选在当前虚拟摆放下仍被碰撞/可达性检查拒绝，不能报告实际模型候选五段规划已通过。

## 官方样例推理指标

`D:\car\tools\graspnet\samples\official-demo\grasps-01.json`，FP32、batch=1、20000 点、seed=0、TF32 关闭。

| 指标 | 实测 |
| --- | --- |
| 首次前向 + 解码 | 1.272963 s |
| 推理 + 碰撞过滤 + 去重 | 2.529205 s |
| 总处理（含加载与几何重建，不含容器启动和文件写入） | 8.206296 s |
| CUDA 峰值 allocated / reserved | 685150208 / 1075838976 字节 |
| 完整场景 / ROI 点数 | 825160 / 688245 |
| 原始 / 碰撞剔除 / 去重剔除 / 超过显示上限 | 251 / 132 / 36 / 33 |
| 最终显示候选 | 50 |

单次冷前向记录，不代表稳定帧率。GPU 显存统计不含驱动上下文。官方样例不是现场 Gemini 六组物体验收。
修复了基础镜像 OpenCV 4.7 的遗留绑定与 headless 4.10 混用，以及 FP32 `0.1` 开口舍入边界。

### 官方场景的不同 ROI

以下是同一官方 RGB-D 场景的不同 ROI，不是现场采集或独立摆放验收。每个结果保留 JSON、完整场景 NPZ、ROI 和日志。

| 结果文件（`official-demo` 下） | ROI 点数 | 候选数 | 前向+解码 / 总处理秒 |
| --- | --- | --- | --- |
| `grasps-bottle-01.json` | 15836 | 4 | 1.023097 / 8.097693 |
| `grasps-bunny-01.json` | 11347 | 29 | 0.994588 / 12.331074 |
| `grasps-cup-01.json` | 2409 | 13 | 1.248455 / 10.791244 |
| `grasps-bunny-full-01.json` | 40194 | 34 | 1.127652 / 9.606637 |
| `grasps-blue-cap-01.json` | 12466 | 2 | 1.191598 / 7.581817 |

这些运行 CUDA 峰值 allocated / reserved 均为 685150208 / 1075838976 字节。
瓶子四个候选开口约 73–83 mm，超过软件夹具 50 mm 上限，拒绝规划；不缩小候选开口以适配夹具。

### 实际模型候选规划的拒绝案例

- `official-bunny-sim-04`：预抓取、接近和闭合通过，挂载目标与障碍发生重叠；明确报告碰撞对。
- 发现独立体素化在 ROI 边界生成重复整盒，已修正为在光学帧按 ROI 裁剪/切分，微米级分离共面边界。
- `official-bunny-sim-05`：修正后挂载状态检查通过，抬起 Cartesian fraction 仍为 0，继续拒绝。
- `official-bunny-full-sim-02`：扩大 ROI 覆盖整个兔子并重新推理；候选 26 开口 40.8 mm。原姿态的预抓取路径被 MoveIt 最终碰撞复核拒绝；180°等价姿态的接近 fraction 0.818，拒绝。
- `official-bunny-full-sim-05`：显式虚构折叠起点通过初始场景检查；候选 26 的接近 fraction 0.800，仍拒绝。该起点只用于 DisplayTrajectory，没有发送关节指令。
- 较高的虚拟桌面摆放还出现肩部/前臂起点碰撞，相关日志保留；虚拟摆放不代表机器人实测外参。

当前单指软件夹具的几何与 GraspNet 平行夹爪代理不同，点云代理也不包含不可见表面或真实接触动力学。
这些因素及场景杂物会限制通过的候选；目前不能把模型候选的全段仿真列为通过，也没有实机抓取成功记录。
后续先用独立摆放的瓶子/纸盒验证，再按实机标定补齐真实指爪几何与 TCP。
规划器只把完整五段结果发布到 `/grasp/display_planned_path`，避免 MoveIt 默认话题中的局部预抓取轨迹被误当作成功结果。

## 真实物体验收清单

| 样本 | 同步快照 | GraspNet / 指标 | 候选回放 | 选中后仿真 |
| --- | --- | --- | --- | --- |
| 官方样例 | 已转换（原文件不含采集时间，明确记为 0） | 已通过 / 指标见上 | 已通过 | 已验证拒绝案例；实际候选完整五段仍待通过 |
| 不透明瓶子 1/2/3 | 待实际摆放采集 | 待 | 待 | 待 |
| 纸盒 1/2/3 | 待实际摆放采集 | 待 | 待 | 待 |

不预设实时性能、真实夹持力或实机成功率。先验收真实点云候选与软件轨迹，再补齐实机标定和控制反馈。

## 复跑软件规划

```bash
source /opt/ros/jazzy/setup.bash
source install/grasp/local_setup.bash
ROS_DOMAIN_ID=77 python3 scripts/graspnet/smoke_sim.py \
  --output /home/xrak/rak/tools/graspnet/sim-smoke-new
```

脚本生成明确标注的合成样本、启动软件夹具、验证五段轨迹/不可达/碰撞并清理进程，输出目录必须不存在。
