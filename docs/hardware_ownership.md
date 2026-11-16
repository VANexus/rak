# 硬件归属（唯一权威说明）

> 目的：避免"桌面机能直接插雷达/深度相机"这个误解。凡是"本机有没有硬件"的判断，以本文为准。

## 两台机器

| | 桌面机（x86，Shizurak） | orin（Jetson，192.168.5.215） |
|---|---|---|
| 角色 | 写上层逻辑、编排、RViz、夹具 | 接真实硬件、跑硬件节点 |
| 构建档位 | `build_profiles.yaml` 的 **desktop** | **jetson** |
| 雷达 `lslidar_driver` | **不构建**（desktop 档跳过） | 已构建，`/dev/wheeltec_lidar` → `/dev/ttyUSB0` |
| 深度相机 `rak_astra_driver` | **不构建**（desktop 档跳过） | 已构建，Orbbec Astra S（`2bc5:0402`，OpenNI/SDK 设备，**不出 `/dev/video*`**） |
| 双目 `rak_stereo_driver` / `rak_pointcloud`/`rak_gpu` | **不构建** | 已构建 |
| IMU `rak_imu_driver` | 可构建，但无设备 | 已构建；I2C `/dev/i2c-7` 0x6B（SPI `/dev/spidev0.0` 读不到 WHO_AM_I） |
| CAN `can0` | **无**（`ip link` 里没有 can 设备） | `can0` UP，ros2_canopen 主站与 `controller_manager` 都跑在这里 |
| `depth_image_proc` / `laser_filters` | 未安装（档位里是 `tolerate_absent`） | 已安装 |

## 结论

- **桌面机不能直接插雷达 / 深度相机 / CAN**，也不要指望在桌面机上 `ros2 launch rak_bringup lidar.launch.py` 出数据。
- 传感器节点（`lslidar_driver`、`rak_astra_driver`、`rak_imu_driver`）**必须在 orin 上启动**，通过 CycloneDDS（domain 47）把 `/scan`、`/camera/depth/*`、`/imu/data_raw` 送到桌面。
- 桌面侧的 `install/` 只包含 desktop 档的包；需要传感器包时是在 **orin 的那份工作区**构建，不是在桌面这份。
- 跨机发现由 `rak_bringup` 的 env-hook 兜底（`CYCLONEDDS_URI` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `ROS_DOMAIN_ID=47`），**source 工作区 overlay 即可，不需要手动 export**。

## 已验证的证据（2026-11-16）

- 桌面机 `ls /dev/serial/by-id` 为空、无 `/dev/ttyUSB*`、`lsusb` 只有一支 UVC 相机；`ip -br link` 无 `can*`。
- orin：`lsusb` → `2bc5:0402 Orbbec ASTRA S`；`/dev/wheeltec_lidar` → `ttyUSB0`；`can0` UP。
- 桌面 `source install/setup.bash` 后 `ros2 topic list` 能看到 orin 发布的 `/rak_probe`（无需任何手动环境变量）。
