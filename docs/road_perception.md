# RoadSeg 视觉导航输入

`rak_road_perception` 在 Jetson 上加载 LFD-RoadSeg TorchScript 模型，发布道路掩码和 RViz 叠加图；可选地将深度点云按分割置信度拆成道路清空射线与非道路障碍点，送入 Nav2 **局部** costmap。LiDAR 仍是主要避障来源，RoadSeg 不发布速度或导航目标。

## 包与话题

| 话题 | 类型 | 用途 |
|---|---|---|
| `/road_perception/road_mask` | `sensor_msgs/Image` (`mono8`) | 道路预测掩码，RViz Image 显示 |
| `/road_perception/overlay` | `sensor_msgs/Image` (`bgr8`) | 原图叠加红色道路区域，RViz Image 显示 |
| `/road_perception/road_clear` | `sensor_msgs/PointCloud2` | 高置信道路地面点，仅用于视觉层内部 raytrace clearing |
| `/road_perception/nonroad_obstacles` | `sensor_msgs/PointCloud2` | 高置信非道路点，经 TF 高度和范围筛选后用于 marking |

Nav2 在 `local_costmap` 中为视觉输入单独建了一个 `ObstacleLayer`，以最大代价合并；它的 clearing 射线不会清除 LiDAR 层的障碍。全局 costmap 和任务/速度控制链不变。视觉点云发布默认关闭。

## Jetson 运行

模型文件不放进 RAK 源码仓库。默认路径沿用车上已部署的室内版：`/tmp/lfd_deploy/lfd_roadseg_indoor.ts`。如果文件不在车上，从开发机复制：

```bash
scp "D:/Desktop/SNN/LFD-RoadSeg-reproduce/deploy_pkg/lfd_roadseg_indoor.ts" xrak:/tmp/lfd_deploy/
```

Jetson 使用 JetPack 已有的 PyTorch、CUDA、OpenCV 和 ROS `cv_bridge`，不安装额外 PyTorch wheel。先在 Jetson 构建工作区，再启动相机和深度点云；`task_navigation.launch.py` 会在同一 ROS domain 启动 SLAM/Nav2、RoadSeg 与 RViz：

```bash
# 在 Jetson 的 rak 仓库根目录构建（工作区规范要求在目标 aarch64 上构建）
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --packages-up-to rak_bringup
source install/architecture/local_setup.bash

# 单独启动 Astra 深度相机和 /camera/depth/points；不重复启动 SLAM、LiDAR 或 RViz
ros2 launch rak_bringup sensors.launch.py \
  camera:=astra lidar:=false imu:=false mic:=false rviz:=false \
  publish_robot_tf:=false

# Astra 彩色图像继续由现有 OpenNI2 feeder 写入共享内存；再启动唯一一套任务/Nav2
ros2 launch rak_bringup task_navigation.launch.py \
  road_perception:=true road_rviz:=true \
  road_input_mode:=shared_memory \
  road_depth_alignment:=none
```

`task_navigation.launch.py` 会启动 SLAM 和 Nav2；不要另起第二套 SLAM/Nav2。上面的 `sensors.launch.py` 只负责相机及深度点云，并关闭自身的机器人 TF 发布，避免和 SLAM 重复发布。运行共享内存模式时，先停掉旧 `jetson_seg_stream.py` 的看门狗，避免重复加载模型和重复占用 GPU。Gemini 或已恢复的 ROS 彩图话题可用 `road_input_mode:=ros_image`，并按实际启动方式避免重复启动相机。

`task_navigation.launch.py` 的默认 `motion_backend:=sim` 只用于软件链路验证，不会驱动实车底盘；本次改动没有打开实车运动。

RViz 面板显示道路掩码、叠加图、非道路候选点、局部 costmap 和 Nav2 路径。`road_rviz:=true` 需要 Jetson 桌面会话；无桌面时可在本机 RViz 加载 `rak_bringup/config/road_navigation.rviz` 并连接机器人 ROS domain。

## 让视觉点云进入局部 costmap

需同时满足：

1. `road_depth_alignment` 已按相机标定设置。`registered` 只适用于深度点云与彩色图逐像素配准的情况；`astra_flip_shift` 使用 Astra S 当前记录的垂直翻转和 `-8 px` 水平偏移；默认 `none` 不生成语义点云。
2. TF 中有实测的 `camera_depth_frame` 到 `base_footprint` 变换。没有变换时节点会跳过点云，不会猜相机安装外参。
3. 先在 RViz 检查图像、深度点云和相机 TF，再显式打开：

```bash
ros2 launch rak_bringup task_navigation.launch.py \
  road_perception:=true road_rviz:=true \
  road_input_mode:=shared_memory \
  road_depth_alignment:=astra_flip_shift \
  road_depth_shift_px:=-8 road_publish_nav_clouds:=true
```

此参数只打开局部视觉点云；它不打开底盘运动。若相机帧、外参或像素配准不正确，先保持 `road_publish_nav_clouds:=false`。

## 推理优化现状

节点默认用 FP32 TorchScript，并在加载时尝试 `torch.jit.freeze` + `torch.jit.optimize_for_inference`；优化不支持时会退回原始 TorchScript。预处理保留现有部署脚本的 RGB 归一化和暗场提亮（灰度均值低于 110 时用 gamma 0.6）。开发机 RTX 4080、480×640 室内帧上的一次测量为 2.51 ms → 1.47 ms，阈值掩码 IoU 0.99983。该结果只筛选了优化方向；Jetson 的端到端速度尚未复测。FP16 与 channels-last 在这次 RTX 4080 对比中更慢，因此没有设为默认。车端要继续以 30 FPS 相机输入和 Nav2 同机负载实测，TensorRT/FP16 需要在 Jetson 上完成模型一致性与资源验收后再启用。
