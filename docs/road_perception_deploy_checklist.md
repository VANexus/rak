# 部署 & 导航对接清单（Jetson 恢复连接后照此执行）

> [AI] 2026-11-13 状态：**车已重装系统**（SSH host key 变化 + 22/5900/8080 全关 = sshd 未装）。
> 网络层在线（ICMP 50% 通）但无登录服务。**离线恢复包已备**：
> `jetson_deploy_offline.tar.gz`（23.7MB，含全部模型/脚本/源码/文档），车边 U 盘拷入解压即重建。

> 前置：Jetson 网络恢复（当前 192.168.5.215 不可达，见 HANDOFF.md §6 P0）。
> 本清单基于 2026-11-11 对 `rak` 仓库（本地 clone）的核对：`rak_road_perception` 包、
> Nav2 `road_perception_layer`、launch/RViz 集成**均已存在于仓库中**，无需新建。

---

## 0. 对接核对结论（已确认，不用再做）

| 项 | 状态 |
|---|---|
| feeder 共享内存格式 vs `roadseg_node` 读取 | ✅ **逐字节匹配**（`<4sIII` 'CAM1'+w+h+size+RGB888；seq 文件） |
| 节点默认参数 vs 我们调优成果 | ✅ 全部一致（adaptive_gamma/110/0.60、th0.40、indoor.ts、shared_memory 模式） |
| 深度-彩色偏移 | ✅ 节点已内置 `astra_flip_shift` 模式（垂直翻转 + `depth_to_color_shift_px: -8`，与实测一致） |
| Nav2 costmap 层 | ✅ `road_perception_layer` 已配进 local costmap（road_clear 清除 + road_obstacles 标记，Max 组合） |
| RViz / launch | ✅ `road_navigation.rviz`、`task_navigation.launch.py road_perception:=true` |
| 唯一缺口 | ❌ feeder 只写彩色；Nav2 点云需要 `/camera/depth/points`（见 §3） |

---

## 1. 部署清单（scp 到车上）

```bash
# 模型（三选一按场景；indoor 为当前默认）
scp deploy_pkg/lfd_roadseg_indoor.ts xrak:/tmp/lfd_deploy/
scp deploy_pkg/lfd_roadseg_v2.ts     xrak:/tmp/lfd_deploy/   # 跨场景备选
scp deploy_pkg/lfd_roadseg_v3.ts     xrak:/tmp/lfd_deploy/   # 边缘感知（camvid 0.937 最优）
# 推流节点（仓库正式版，优先于我们的临时版）
scp deploy_pkg/jetson_seg_stream.py  xrak:/tmp/oni_old/
# ROS 桥（feeder 共享内存 -> ROS Image 话题）
scp deploy_pkg/shm_to_ros.py         xrak:/tmp/oni_old/
# 仓库包（若车上 src/rak_road_perception 缺失；本地 D:\Desktop\rak 已有）
scp -r "D:/Desktop/rak/src/rak_road_perception" xrak:~/rak/src/
```

## 2. 车上启动序列（按序执行）

```bash
# ① 采集供帧（导航版：color+depth 双流，看门狗）—— feeder_nav.cpp 在 deploy_pkg/，先上传编译：
#    scp deploy_pkg/feeder_nav.cpp xrak:/tmp/oni_old/ && ssh xrak 'cd /tmp/oni_old && g++ -O1 -I Include feeder_nav.cpp -L . -lOpenNI2 -Wl,-rpath,/tmp/oni_old -o feeder_nav'
#    （watchdog 循环里把 ./feeder 换成 ./feeder_nav）
ssh xrak 'cd /tmp/oni_old && setsid nohup bash -c "cd /tmp/oni_old && while true; do ./feeder >> /tmp/feeder.log 2>&1; sleep 2; done" >/dev/null 2>&1 </dev/null &'

# ② 分割推流（RViz 用；看门狗）
ssh xrak 'cd /tmp/oni_old && setsid nohup bash -c "cd /tmp/oni_old && while true; do python3 jetson_seg_stream.py >> /tmp/seg_stream.log 2>&1; sleep 3; done" >/dev/null 2>&1 </dev/null &'

# ③ ROS 桥（供 roadseg_node 的 ros_image 模式；可选）
ssh xrak 'source /opt/ros/jazzy/setup.bash && cd /tmp/oni_old && setsid nohup python3 shm_to_ros.py --depth > /tmp/shm_bridge.log 2>&1 < /dev/null &'

# ④ RoadSeg ROS 节点（仓库正式版；shared_memory 模式下③可省）
ssh xrak 'source /opt/ros/jazzy/setup.bash && source ~/rak/install/architecture/local_setup.bash && export ROS_DOMAIN_ID=47 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp && setsid nohup ros2 run rak_road_perception roadseg_node --ros-args -p model_path:=/tmp/lfd_deploy/lfd_roadseg_indoor.ts -p input_mode:=shared_memory > /tmp/roadseg.log 2>&1 < /dev/null &'

# ⑤ Nav2 + 分割层
ssh xrak 'source /opt/ros/jazzy/setup.bash && source ~/rak/install/architecture/local_setup.bash && export ROS_DOMAIN_ID=47 && ros2 launch rak_bringup task_navigation.launch.py road_perception:=true'
```

## 3. 对接导航的两个前置缺口

### 缺口 A：深度点云（`road_obstacles`/`road_clear` 的数据源）

`nav2_params.yaml` 的 `road_perception_layer` 订阅 `/road_perception/road_clear` 与
`/road_perception/nonroad_obstacles`（PointCloud2）——由 `roadseg_node` 在
`publish_nav_clouds:=true && depth_alignment∈{registered,astra_flip_shift}` 时发布，
输入是 `pointcloud_topic: /camera/depth/points`。

**状态（2026-11-13）：彩色链路已补齐，深度链路代码已备**：
- `deploy_pkg/feeder_nav.cpp` = 双流版 feeder（color→cam_frame.bin + depth→cam_depth.bin，
  上车编译一条命令，见 §2 ①）；`shm_to_ros.py --depth` 发 `/camera/depth/image_raw`
- 还差一环：depth image → PointCloud2 需要 `depth_image_proc`
  （上车验证：`ros2 pkg list | grep depth_image_proc`，缺则 apt 安装 ros-jazzy 版）
- 注意：feeder_nav 写的是**原始倒图深度**；点云 frame 为 `camera_depth_optical_frame`
  （URDF 已补倒装 rpy=pi 假设）——点云左右是否镜像需上车用 RViz 对照激光雷达验证，
  若镜像改 URDF 的 `cam_roll`（参数化，一处即改）
- 方案 b（astra_bridge_node 换老 OpenNI2，HANDOFF §6 P3）仍为仓库级正式方案
- [历史] 原缺口记录：feeder 只写彩色共享内存，没有深度点云源。补齐两选一：
- **方案 a（推荐）**：扩展 feeder 读深度流（data_collect.cpp 已有成对采集先例），
  写 `/dev/shm/cam_depth.bin` + `cam_depth_seq`，`shm_to_ros.py --depth` 转 ROS 话题，
  再加 `depth_image_proc`（apt: ros-jazzy-depth-image-proc）转 PointCloud2
- **方案 b**：`astra_bridge_node`（rak_astra_driver）换老 OpenNI2 后原生发深度/点云
  —— 即 HANDOFF §6 P3，仓库级正式方案

### 缺口 B：TF 标定

`roadseg_node` 点云路径需要 `camera_depth_frame → base_footprint` 的 TF
（`base_frame` 参数）。检查 `rak_description` 的 URDF 是否已含 ASTRA S 深度相机的
静态 TF；没有则补 `<static_transform_publisher>`（安装位置实测后填 x/y/z/rpy）。

### 缺口 C：语义点云默认禁用（设计如此）

`AGENTS.md` 规定："semantic point clouds stay disabled until camera alignment and
TF calibration are confirmed"。解锁步骤：
1. `depth_alignment:=astra_flip_shift`（倒装+偏移模式，节点已实现）
2. 确认 `depth_to_color_shift_px`（当前默认 -8，来自 2026-11-08 实测；建议用 20 帧边缘
   匹配复核）
3. TF 可用后，`publish_nav_clouds:=true` 才会有实际点云输出

## 4. 验证检查点（按序）

```bash
# V1 供帧
ssh xrak 'cat /dev/shm/cam_seq'        # 数字持续增长
# V2 推流
ssh xrak 'ss -tn | grep 9001'          # ESTAB（viewer 连接时）
# V3 ROS 桥/节点
ssh xrak 'ros2 topic hz /camera/color/image_raw'          # ~30Hz
ssh xrak 'ros2 topic hz /road_perception/road_mask'       # ~30Hz
# V4 Nav2 层
ros2 topic echo /road_perception/road_clear --once        # 有 PointCloud2 输出
# V5 端到端
RViz 打开 road_navigation.rviz，看 mask/overlay；移动相机验证红色区域跟随
```

## 5. 模型选型速查（model_path 参数）

| 场景 | 模型 | 依据 |
|---|---|---|
| 固定室内场地（本房间） | `lfd_roadseg_indoor.ts` | indoor IoU 0.788（40 帧微调） |
| 跨场景/室外/路口 | `lfd_roadseg_v3.ts` | camvid IoU 0.937、BF1 0.6575（边缘感知最优） |
| 对照/回退 | `lfd_roadseg_exp1.ts` / `v2.ts` | 复现基线 / 泛化对照 |

⚠️ 室内模型标签含伪标签陷阱（HANDOFF §3.4）：黑色地毯区域可能漏检；
   几何先验（`y>0.42H` 之外不输出）已在部署端兜底。

## 6. 车边操作步骤（系统重装后，显示器键盘登录车）

```bash
# 1) SSH 服务 + 公钥（恢复远程连接）
sudo apt update && sudo apt install -y openssh-server
sudo systemctl enable --now ssh
mkdir -p ~/.ssh
echo 'ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIEVKl5KoYjjVfy1c/MESNKH4pIuGXRcrAhTSsnJEzL7E w5555wdnmd@gmail.com' >> ~/.ssh/authorized_keys
chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys
ip addr show   # 确认新 IP（可能不再是 192.168.5.215）

# 2) U 盘拷入离线包并解压（U 盘挂载点按实际改）
mkdir -p /tmp/restore && tar -xzf /media/xxx/jetson_deploy_offline.tar.gz -C /tmp/restore

# 3) 按包内 HANDOFF.md §7.1 重建：老 OpenNI2 栈、feeder_nav 编译、看门狗启动
# 4) ROS/torch 若也被清：重装按 HANDOFF.md §2（ros-jazzy + torch 2.12+cu132 + cv_bridge）
```
