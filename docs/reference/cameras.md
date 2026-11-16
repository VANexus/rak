# RGB-D Camera Backends

> 状态：现行 · 最后更新：2026-11-16

`rak_bringup` owns the selectable RGB-D camera entry point. It starts one
backend at a time and keeps camera SDKs out of navigation and perception
consumers.

## Select a camera

```bash
ros2 launch rak_bringup sensors.launch.py                       # Gemini 335L, wifi profile
ros2 launch rak_bringup sensors.launch.py camera_profile:=full  # 1280x800@30
ros2 launch rak_bringup sensors.launch.py camera_ir:=left       # + 左 IR
ros2 launch rak_bringup sensors.launch.py camera:=astra         # Astra S 路径
```

The two cameras are swapped on the same mount, so the backend is a launch
argument (`camera`), never a compile-time choice. `gemini335l` is the default.

```bash
ros2 launch rak_bringup depth_camera.launch.py camera:=gemini335l
ros2 launch rak_bringup depth_camera.launch.py camera:=gemini335l serial_number:=YOUR_SERIAL
```

Profile YAMLs live in `src/rak_bringup/config/` and can be overridden with
`config_file_path:=/path/to/profile.yaml` on `gemini335l.launch.py`.

## What the device is (Gemini 335L, verified 2026-11-16)

| 项 | 值 |
|---|---|
| USB | `2bc5:0804` Orbbec Gemini 335L, `bcdDevice 0.01`, 字符串序列号 `CP2G853000G0` |
| 链路 | SuperSpeed，`/sys/.../speed = 5000`（驱动自报 `usb connect type: USB3.2`） |
| 枚举 | 8 个 UVC 1.50 节点（`/dev/video2..9` + 2 个 media 节点），`uvcvideo` 绑定 |
| 协议 | 图像走标准 UVC；IMU、元数据、曝光/滤波等控制走厂商私有通道（`bInterfaceClass 0xff`），由 Orbbec SDK v2 封好，**不是**独立的 V4L2 设备 |
| 板载 IMU | 机身 6 轴 IMU，只能经 SDK 私有协议读取（不在 `lsusb` 里单独出现，也没有可直读的 I2C/SPI 节点） |
| 流 | depth Y16、color RGB888/BGR/MJPG/YUYV、IR（左右各一路），支持 424x240 / 480x270 / 640x360 / 640x400 / 640x480 / 848x480 / 1280x720 / 1280x800；**没有 320x240** |

## Driver Ownership

| Backend | ROS driver | Ownership |
|---|---|---|
| Gemini 335L | Orbbec `orbbec_camera` 2.9.3 (SDK v2) | 上游官方 ROS 2 wrapper；本仓库只放 launch 适配与 profile |
| Astra | `rak_astra_driver` | 既有 Astra 专用 bridge（SDK v1） |

Gemini 走官方二进制包 `ros-jazzy-orbbec-camera`（orin 已装）；本仓库不 vendor
SDK 或上游源码。开发机（lyrical）没有这个包，`gemini335l.launch.py` 会直接给出
一句人话的错误（而不是 ament 堆栈），此时用 `camera:=astra` 或把 ROS 指向 orin。

### udev：必须有 0804 那条规则

`/etc/udev/rules.d/56-orbbec-usb.rules`（旧 SDK 安装版）**没有** 0804，设备节点
归 `root:root`，libusb 报 `Access denied`，SDK 枚举出的序列号为空，驱动按 SN 选
设备就永远 "Device with serial number ... not found"——看起来像没插相机。

修法：装官方包自带的那份（含 0804），由 `scripts/system/rak-system-baseline.sh`
自动做：

```bash
cp /opt/ros/jazzy/share/orbbec_camera/udev/99-obsensor-libusb.rules /etc/udev/rules.d/
udevadm control --reload-rules
udevadm trigger --action=change --subsystem-match=usb --attr-match=idVendor=2bc5
```

装完应有 `crw-rw-rw- root:video /dev/bus/usb/00X/00Y` 和 `/dev/Gemini_335L` 软链。
**不要**对整个 USB 子系统发 `--action=add`：会把 USB 网卡一起重新枚举，orin 直接掉网。

### 物理连接：直连 orin 的 USB3 口

相机是直接插在 orin 的 USB3 口上的。`lsusb -t` 里看到的
`Bus 002 → Port 001 (Hub, 10000M) → Port 003 (相机 5000M)` 中的那个
`2109:0822 VIA Labs USB3.1 Hub` 是**载板自带的板载 Hub**，不是外接扩展坞，
相机路径 `2-1.3` 就代表"orin 的 USB3 口 + 板载 Hub + 相机"，属于正常拓扑。

`tegra-xusb` 报 `ERROR Transfer event TRB DMA ptr not part of current TD` 与
`usb 2-1.3: USB disconnect` + 重新枚举，是 xHCI 驱动在这块 Jetson 上的问题，
**不是**"相机挂了外置 Hub"导致的；2026-11-16 上午整机两次硬重启也发生在这个
拓扑下，换口/换线未做对比实验。遇到这类日志先看驱动是否自行恢复出流，
不要误判成接线方式问题。

## ROS Interface

| Stream | Topic | Frame |
|---|---|---|
| RGB | `/camera/color/image_raw` | `camera_color_optical_frame` |
| RGB info | `/camera/color/camera_info` | — |
| Depth | `/camera/depth/image_raw`（`16UC1`，毫米） | `camera_depth_optical_frame` |
| Depth info | `/camera/depth/camera_info` | — |
| IR（`camera_ir:=left|right|both`） | `/camera/ir/left/image_raw`, `/camera/ir/right/image_raw` | `camera_left_ir_optical_frame` / `camera_right_ir_optical_frame` |
| 点云 | `/camera/depth/points`（`PointCloud2`，orin 上 `depth_image_proc` 生成） | `camera_depth_optical_frame` |
| IMU（accel+gyro 合并） | `/camera/gyro_accel/sample`（`sensor_msgs/Imu`，实测 ≈187 Hz） | `camera_accel_gyro_optical_frame` |
| IMU 单轴 | `/camera/accel/sample`, `/camera/gyro/sample`（仅当 `enable_sync_output_accel_gyro:=false` 时出现） | `camera_accel_optical_frame` / `camera_gyro_optical_frame` |

**话题名以实测为准**：2.9.3 开 `enable_sync_output_accel_gyro` 后，合并话题叫
`<ns>/gyro_accel/sample`，不是很多教程写的 `<ns>/imu/sample`；此时单轴的
`accel/sample`、`gyro/sample` 不发布，只有 `accel/imu_info`、`gyro/imu_info`
两个元数据话题。frame 同理是 `camera_accel_gyro_optical_frame`（不是
`camera_imu_optical_frame`）。查法：`ros2 topic list | grep -i imu` 加
`ros2 topic echo --once` 看 `header.frame_id`。

Astra 沿用同名图像话题，但没有彩色、也没有 IMU；点云同样由 `depth_image_proc`
生成，所以两个 backend 的话题契约一致。深度注册（`depth_registration`）保持
关闭：深度图在深度光学坐标系下给出，彩色与深度的对应关系用 `CameraInfo` 和
出厂外参，不要假设像素对齐。

### 两个 profile

| profile | depth | color | 用途 |
|---|---|---|---|
| `wifi`（默认） | 424x240 Y16 @10 ≈ 2.0 MB/s | 424x240 RGB @5 ≈ 1.5 MB/s | orin 出流、开发机 RViz 远程看（链路实测约 4 MB/s） |
| `full` | 1280x800 Y16 @30 | 1280x800 RGB @30 | 本机显示或有线千兆；1280x800 RGB 单独就 ~92 MB/s 的拷贝 |

彩色默认 5 fps 是链路预算的结果，不是能力上限。IR 默认关：每路都是又一张
16-bit 图，导航用不上，深度图已经覆盖同一几何。

`wifi` 档合计 ≈3.5 MB/s，实测**一个**远程 RViz 订阅者能稳定跑（depth 10.1 Hz、
color 4.8 Hz、15 s 无空洞）；但每个远程订阅者都会独立拉一份全量流，两个实例就
≈7 MB/s，会开始丢整帧、面板停在最后一帧。要多开看图时先把 color 降到 2 fps
或只留 depth，别指望链路自动分流。

点云由 orin 上的 `depth_image_proc` 生成，跨机显示时它是额外 ~3 MB/s，所以
`sensors_rviz.rviz` 里点云 display 默认关。上游 SDK 自建点云默认关；
`enable_point_cloud` **不要写进 profile YAML**——上游 launch 的 YAML 会盖掉
launch 参数，写进去就等于把 `enable_point_cloud:=true` 永久废掉。

## IMU：两路，各管一件事

| 话题 | 源 | 频率 | frame | 角色 |
|---|---|---|---|---|
| `/imu/data_raw` | Jetson 直连 LSM6DSRTR（`rak_imu_driver`，I2C） | 1 kHz | `imu_link` | 航向/EKF 权威源，零偏在驱动里补（见 `imu.md`） |
| `/camera/gyro_accel/sample` | Gemini 335L 机身 IMU，经 SDK | 200 Hz（实测 ≈187 Hz） | `camera_accel_gyro_optical_frame` | 交叉校验、可视化；跟相机同源，换机/换相机不影响航向链 |

**两个话题名不同，不存在同话题多发布者**；不要 remap 到同一个话题，也不要让
EKF 同时吃两路（同机两路 IMU 融合会互相拉扯）。

约定：

- `orientation` 不提供 → `orientation_covariance[0] = -1`，其余为 0（REP-145）。
- `linear_acceleration_covariance` / `angular_velocity_covariance` 的对角项由驱动用
  `linear_accel_cov`（默认 0.01）填。这是驱动的保守估计，**不是** Allan 方差测出来的值；
  需要更严的协方差就改 profile 里的 `linear_accel_cov`，别改上游代码。
- 量程/频率：`accel 4g @200Hz`、`gyro 1000dps @200Hz`，SDK 侧 `*_data_correction` 打开
  （厂商温度/非线性校正），所以这里的值已经过校正，不要再套一层比例。
- 轴向：`camera_accel_gyro_optical_frame` 遵循相机光学约定（x 右、y 下、z 前；
  该 frame 就是按 REP-103 光学约定相对 `camera_link` 旋转出来的，实测 TF 为
  roll=-90°、yaw=-90°，与 `camera_color_optical_frame` 同姿态）。静止时实测
  `linear_acceleration` 约为 `(-0.33, -9.86, 0.14)`——重力落在 -y，符合"y 朝下"，
  也说明相机是正装的，没有倒置/侧装。
  IMU 相对深度/彩色的安装外参由出厂标定给出（`enable_publish_extrinsic:=true`），
  驱动 TF 链 `camera_link → camera_accel_gyro_optical_frame` 由相机侧发布，
  不要在本仓库再补一份静态 TF。整机层面的位置是 `base_link → camera_link`
  由本仓库静态 TF 给（实测 `camera_accel_gyro_optical_frame` 在 base_link 下
  `(0.136, -0.008, 0.071) m`），换相机时只需要改这一层。
- 标定（需要做时）：静止放置 ≥5 min 预热 → 录 60 s `/camera/gyro_accel/sample` →
  `gyro_bias = mean(angular_velocity)`、`accel_bias = mean(linear_acceleration) - R*(0,0,9.80665)`，
  其中 R 由静止时的重力方向反算 roll/pitch；回代后再验 `|gyro| < 0.005 rad/s`、
  `|a| = 9.807 ± 0.05`。方法与 LSM6DSR 那套一致，见 `imu.md`；相机 IMU 的零偏**不**写进
  `i2c_imu` 的参数，两者各自独立。

## 验证记录

- 2026-11-16（设备/驱动）：枚举 `2bc5:0804`、USB3.2 5000 Mbps、8 个 UVC 节点；udev 修复后
  驱动按 SN `CP2G853000G0` 匹配成功；depth 424x240@10 `16UC1`（203,520 B/帧）、color
  424x240@5 `RGB888`（305,280 B/帧）、IMU `4g/1000dps @200Hz` 均起流。
- 2026-11-16（跨机实测，开发机侧收流，`ROS_DOMAIN_ID=47` + `rmw_cyclonedds_cpp`）：
  - `/camera/depth/image_raw` 10.13 Hz，`stamp` 与墙钟差 -0.26 s；非零像素 47%，
    中位深度 3379 mm（p10 3211 / p90 3700），值域合理。
  - `/camera/color/image_raw` 4.77 Hz、1.46 MB/s，15 s 内无 >0.5 s 空洞；
    连续帧逐像素比对 97–99% 像素在变——数据是活的，不是冻结帧。
  - `/camera/gyro_accel/sample` ≈187 Hz，`orientation_covariance[0] = -1`，
    `angular_velocity_covariance` / `linear_acceleration_covariance` 对角 0.01，
    `frame_id = camera_accel_gyro_optical_frame`，发布者 1 个（RELIABLE）。
  - TF：`base_link → camera_accel_gyro_optical_frame` = (0.136, -0.008, 0.071) m，
    四元数 (-0.5, 0.5, -0.5, 0.5)。
  - 本机 RViz（`sensors_rviz.rviz`）：彩色面板实时刷新，相机 IMU 显示为橙色加速度箭头
    （键名必须用 `Axes properties` / `Acceleration properties` 那套，见 rviz 文件注释）。
- 带宽注意：WiFi 上**每个远程订阅者都要独占整条流**。同时开两个 RViz 实例（各订阅
  depth+color）时，两路合计需求 ≈7 MB/s 超过链路 ≈4 MB/s，表现为面板停在最后一帧
  （就是之前"静态预览帧"的观感）。排查时先确认只有一个订阅者：
  `ros2 topic info /camera/depth/image_raw`。
- 2026-11-16（USB 稳定性）：相机直连 orin 的 USB3 口（`2-1.3`，`2-1` 是载板板载
  VIA Hub，见上文）。起流瞬间 `tegra-xusb` 报过一次 `TRB DMA` + 设备重枚举
  （Device 3 → 4），随后驱动自行恢复、持续出流；但前一 boot 同一接线方式下曾连续
  报错并导致整机硬重启两次。这是 xHCI/内核侧的问题，不是接线方式问题。
- 2026-10 记录（同一台 335L、同一接线）：RGB/depth 各约 29.96 Hz、IMU 约 200 Hz、
  RGB/depth 时间戳偏差约 5.41 ms——说明硬件与驱动本身没问题。

## 编译

| 机器 | 命令 | 结果 |
|---|---|---|
| orin (jazzy) | `colcon build --symlink-install --packages-select rak_bringup` | 通过（0.51 s） |
| 开发机 (lyrical) | 同上 | 本机 install 树不完整（缺 `lslidar_msgs`/`rak_gpu`/`rak_astra_driver` 等），`rak_bringup` 因依赖检查失败；改动仅为 Python launch + YAML，已用 `py_compile` / `yaml.safe_load` 与 `ros2 launch --show-args` 校验，缺 `orbbec_camera` 时按预期给出提示 |
