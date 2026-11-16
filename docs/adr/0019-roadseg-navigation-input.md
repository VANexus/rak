# ADR-0019: Jetson 道路分割作为 Nav2 局部视觉输入

- 状态: 已接受
- 日期: 2026-11-11
- 相关: `src/rak_road_perception`、`src/rak_bringup`、`docs/road_perception.md`、LFD-RoadSeg 交接文档

## 背景

LFD-RoadSeg 已有可部署的 TorchScript 权重和 Jetson 推理流程；小车需要在 RViz 检查道路分割，并为导航提供视觉信息。当前 Astra 彩色图像可由现有 feeder 写入共享内存，统一的彩色 ROS topic 尚未在车端验收。RGB-D 像素配准和相机到车体外参也必须实测，不能从模型输出臆造。

本 ADR 定义模型在机器人仓库中的职责和 Nav2 接口，不代表车端编译或安全验收已完成。Nav2 Jazzy `ObstacleLayer` 支持 `PointCloud2`、独立 marking/clearing 配置及 `combination_method`（来源：[Nav2 Jazzy Obstacle Layer 参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)）。

## 决策

1. 在 `src/rak_road_perception` 建立独立的 Python ROS 2 包，加载 Jetson 已提供的 PyTorch 与 TorchScript；输入支持 ROS `Image` 和现有 Astra 共享内存帧。
2. 发布标准 `sensor_msgs/Image` 道路掩码与叠加图，供 RViz 查看；本包不输出导航目标、速度或电机命令。
3. 仅当显式配置图像/深度配准方式且存在相机深度帧到 `base_footprint` 的实测 TF 时，才发布语义 `PointCloud2`：高置信道路地面点只用于 clearing，高置信非道路点只用于 marking。
4. 由唯一组装层 `rak_bringup` 启动节点，并在 Nav2 **局部** costmap 配置独立 `ObstacleLayer`。视觉层使用 max 合并方法，保留 LiDAR 层的障碍；视觉点云与 RViz 启动均为可选项，语义点云默认关闭。
5. 模型权重放在车端部署目录，不提交到 ROS 仓库；不增加自定义消息或 Nav2 插件。

## 理由

- 分割推理与任务决策职责不同，放进 `rak_agent` 会越过已定义的决策/执行边界；独立包可单独部署和观测。
- 标准 `Image` 与 `PointCloud2` 保持冻结消息契约不变；Nav2 已支持深度点云的 marking 与 clearing。
- 视觉感知可以补充可通行区域和非道路障碍，但模型误判不能清除 LiDAR 层障碍或改变导航目标。
- 共享内存输入复用已存在的摄像头路径；ROS `Image` 输入为后续恢复统一相机 topic 留出接口。

## 后果

**正面：** 模型结果可通过 RViz 直接检查；视觉输入与导航参数分层；接入失败或未校准时，默认不向 costmap 注入语义点云。

**负面：** Jetson 需同时承担 PyTorch、OpenCV、图像处理和 Nav2 的内存/GPU 开销；颜色分割与深度几何的时间同步、配准及 TF 标定会影响点云质量；固定阈值可能随照明和场景变化。

## 被否决的方案

- **把模型并入 `rak_agent`：** 该包负责决策，不是高频图像推理或传感器适配层，会混淆职责边界。
- **让视觉层替代 LiDAR 或直接控制 `/cmd_vel`：** 分割模型没有足够几何/安全保证，不适合作为唯一障碍来源或运动控制器。
- **新建自定义 costmap 插件和消息：** 当前按置信度筛选深度点云即可表达道路清理射线与非道路障碍，标准 Nav2 layer 和消息足够。
- **立即转 TensorRT/FP16：** 目前只完成本机 GPU 的优化对比，没有 Jetson parity、内存和端到端负载数据；先保留 FP32 TorchScript 路径。

## 待闭合项

1. Jetson 网络恢复后，在 JetPack/ROS 2 Jazzy 上构建、安装并确认 PyTorch、cv_bridge、OpenCV 依赖可用。
2. 确认相机 feeder 与 ROS `Image` 的图像方向、帧率、时间戳和颜色空间，并在 RViz 检查掩码、叠加图和深度点云。
3. 实测相机到 `base_footprint` 外参及 RGB-D 像素配准；之前不得启用 `road_publish_nav_clouds`。
4. 测量 Jetson 上模型推理和全链路帧率、GPU/内存占用、延迟及模型效果；当前 RTX 4080 结果不代表车端性能。
5. 在静止/仿真环境先观察局部 costmap 的 marking、clearing 和 LiDAR 障碍保留，再按硬件验收流程开展低速导航验证。
