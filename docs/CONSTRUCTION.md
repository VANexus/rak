# CONSTRUCTION — 从零构建与部署

本文回答：**要把这套系统从空板子搭到能跑，按什么顺序做什么。**

与其它文档的分工：
`README.md` 是工程入口（有哪些包、怎么构建）· `docs/architecture.md` 是系统结构 ·
`docs/adr/` 是决策理由 · `docs/hardware_acceptance.md` 是上车门槛 ·
**本文是装配顺序与部署检查单**。

> 阅读约定：标 **[已实现]** 的是仓库里已有代码/脚本；标 **[待实现]** 的是设计已定但代码未落地；
> 标 **[需实测]** 的是占位值，必须用测量替换后才能上车。本文不把"构建通过"当作"能跑"。

---

## 0. 拓扑

```
        ┌──────────────── Jetson Orin Nano (4GB) ────────────────┐
        │  ROS 2 Jazzy · JetPack 7.2 (CUDA 13.2 / TRT 10.16 / VPI 4.1.4)  │
        │  ros2_control · Nav2 · SLAM · MoveIt2 · DeepStream · rak_agent  │
        └───┬──────────────┬──────────────┬──────────────┬───────┘
     USB  │          USB   │        USB   │        40-pin │ SPI
   ┌──────▼──────────────┐ ┌──────▼───────┐ ┌─────▼──────┐ ┌────▼──────────┐
   │  F407（唯一 MCU）    │ │ LSLIDAR N10P │ │ Astra 深度 │ │ LSM6DSRTR IMU │
   │  4×25 kHz 轮 PID     │ │ 2D 雷达      │ │ / 双目     │ │ → EKF 融合    │
   │  + 6× 舵机 PWM @50Hz │ └──────────────┘ └────────────┘ └───────────────┘
   │  + 板载 IMU（轮级）  │
   └─────────────────────┘
```

**关键约束**：

- **臂板已烧毁** → 只剩**一块** F407，同时管 4 轮 + 6 舵机。**CAN 链路失效**，Jetson 只走 USB。
- **麦轮 IK 在上位机**（`mecanum_drive_controller`），F407 只做每轮 PID + 舵机 PWM（ADR-0007）。
- **机体航向权威 = Jetson 的 LSM6DSRTR**；F407 板载 IMU 只做**轮级**自稳定，不参与 EKF。

---

## 1. 硬件清单

| 部件 | 型号 / 规格 | 接口 | 状态 |
|---|---|---|---|
| 上位机 | Jetson Orin Nano **4GB**（p3767-0004） | — | [已实现] |
| **下位机 MCU** | STM32F407VET6 · M4F（C30D 板） | **USB-UART 921600, 100 Hz** | 固件在 `vehical_rak_chassis`（**开发中**） |
| 机械臂 | **标准 6 轴** + **舵机夹爪**（第 7 个 PWM 舵机）+ 末端摄像头 | 经下位机（7 路舵机 PWM） | **[未成形]** 见 ADR-0008 |
| 雷达 | WHEELTEC LSLIDAR **N10P**（Leishen，2D） | USB 串口 460800 | [已实现] |
| 深度相机 | Orbbec **Gemini 335L**（当前优先）+ Astra（可替换后端） | USB | [实机 RGB-D/IMU 已验收；`base_link` 外参待标定] |
| 双目 | USB SBS MJPEG 1280×720@30 | USB | [已实现] |
| 末端摄像头 | 型号未定（eye-in-hand） | USB? | **[待实现]** 见 ADR-0008 |
| 语音模块 | 型号未定 | ? | **[待实现]** 见 TODO P2-6 |
| IMU | ST **LSM6DSRTR** | 40-pin **SPI**（mode 3, 10 MHz） | **[待实现]** 见 §5 |
| 触摸屏 | 1024×600 | — | 与相机争 CMA，见 §6 |

> ⚠️ **臂板已烧毁**：现在只有**一块** F407，同时驱动 4 轮 + **7 路舵机**（6 关节 + 1 夹爪）。
> 不要再按"两块 MCU + CAN"理解拓扑。

---

## 2. 主机（Jetson）准备

### 2.1 基线

- JetPack 7.2 / L4T R39.2.1 / Ubuntu 24.04.5 / kernel 6.8.12-tegra（`PREEMPT`，**非 RT**）。
- ROS 2 Jazzy + `colcon`。
- **不需要实时内核**：闭环在 MCU，Jetson 只做规划与 IK，出厂 `PREEMPT` 足够。
  ⚠️ 不要装 apt 的 `linux-image-realtime`（无 Tegra 支持）。

### 2.2 主机侧已做变更（2026-10-05，非仓库代码）

这些是板级状态，换板/重装后需要重做。**不在 git 里，本文是唯一记录。**

| 变更 | 内容 | 影响 |
|---|---|---|
| SSH 加固 | 禁用密码登录，仅公钥；`PermitRootLogin no` | 改前确认 `authorized_keys` 非空 |
| 移除 snap | snapd + firefox/thunderbird + 44 个无关包移除 | 释放 ~4 GB；**无浏览器** |
| 内存 | zram（zstd, 50% ≈1.7 G, prio 100）+ 4 G swapfile | **改变了 CLAUDE.md 记录的内存前提**：现在有 5.7 G swap |
| 电源 | `nvpmodel -m 0`（10 W）+ `jetson-clocks` 开机自启 | 锁频会改变热包线，基准不可跨模式比较 |
| UART | udev `/dev/robot_uart` → `ttyTHS1`（40-pin） | ⚠️ **与底盘链路无关**（底盘走 USB）。按 ADR-0007 应清理或明确其归属 |

> ⚠️ 移除 snap 后 `CLAUDE.md` 的 "3.4 GB 可用内存、不能常驻所有模型" 结论仍然成立，
> 但"内存不足就 OOM"的紧迫性下降（有 swap 兜底）。**swap 不是实时性保证**，只是防 OOM。

### 2.3 运行模式（CMA 约束）

桌面与相机争同一块 256 MiB CMA 池。**登录的 GNOME 会把 `CmaFree` 打到 ~0**，
导致双目 `VIDIOC_REQBUFS` 失败 —— 看起来像相机故障，其实是内存决策。

```bash
sudo scripts/rak-mode.sh headless   # 相机/SLAM/Nav2 工作用这个
sudo scripts/rak-mode.sh desktop    # 需要 RViz/MoveIt GUI 时
sudo scripts/rak-mode.sh status
```

自动登录是**故意关掉**的，回桌面要在触摸屏输一次密码。

---

## 3. 工作区构建

在只加载 ROS 基础环境的新终端、仓库根目录执行（**不要** source 旧 `install/setup.bash`）：

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src brain --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash
python3 scripts/check_workspace.py --installed
```

只开发决策层可跳过 GPU 包：`--packages-up-to rak_agent`。

**两道必过的检查**（`CLAUDE.md` 规定）：

```bash
colcon test --build-base build/architecture --packages-select rak_description rak_moveit_config
env -u PYTHONPATH brain/.venv/bin/python brain/tests/test_ai.py
```

`env -u PYTHONPATH` 对 `brain/.venv` 内的命令是**强制**的：`~/.bashrc` 无条件 source ROS，
会把 `/opt/ros/jazzy/.../site-packages` 注入 venv，破坏隔离。它**不**适用于 `ros2 run/launch`。

---

## 4. 传感器 bringup

各自独立启动，**不要全开**：

```bash
ros2 launch rak_sensor_bringup lidar.launch.py     # N10P → /scan (frame laser_link)
ros2 launch rak_sensor_bringup depth_camera.launch.py  # 默认 Gemini 335L
ros2 launch rak_sensor_bringup depth_camera.launch.py camera:=astra
ros2 launch rak_sensor_bringup stereo.launch.py device:=/dev/video0
```

雷达的 `lidar_name` 必须是 **`N10_P`**（108 B @ 460800）。写成 `N10` 会导致 CRC 不匹配 ——
这是实测结论，见 `config/lslidar_n10.yaml` 的注释。串口用 udev 别名 `/dev/wheeltec_lidar`，
**不要**用 `/dev/ttyUSB0`（雷达适配器和底盘都可能占 ttyUSB）。

---

## 5. IMU（LSM6DSRTR）— 待实施

**最高性能路线**：内核 `st_lsm6dsx` 驱动（IIO + FIFO + DRDY 中断 + 硬件时间戳），
**不是**用户态 spidev 轮询。

已备好但**未应用**：`scripts/lsm6dsr-overlay/`

```bash
sudo bash scripts/lsm6dsr-overlay/apply.sh    # 编译+fdtoverlay 验证+装 dtbo+改 extlinux
sudo reboot
sudo bash scripts/lsm6dsr-overlay/verify.sh   # 期望 spi0.0 -> st_lsm6dsx_spi
# 回滚
sudo bash scripts/lsm6dsr-overlay/revert.sh && sudo reboot
```

接线：SPI `19/21/23/24`，**DRDY(INT1) → 40-pin pin15**（`PN.01` = gpiochip0 line 85）。

- overlay 已通过 `dtc` 编译 + `fdtoverlay` 合并验证（`&gpio` fixup 正确解析）。
- **[需实测]** 尚未真实重启验证 —— 硬件未接。换 DRDY 引脚要改 `.dts` 里
  `interrupts = <line 1>` 与 `nvidia,pins` 两处（line = GPIO 号 − 512，查
  `/sys/kernel/debug/gpio`）。
- 驱动就绪后，ROS 侧需要一层 IIO→`sensor_msgs/Imu` 的桥接节点 **[待实现]**，
  再喂给 `robot_localization`（见 ADR-0002）。

---

## 6. 底盘与机械臂链路（ros2_control）

**目标架构**（ADR-0007）：一个 `hardware_interface::SystemInterface` 插件同时暴露
4 轮（velocity 命令）+ 6 舵机（position 命令），走 USB。

```bash
ros2 launch rak_bringup system.launch.py     # controller_manager + 各 spawner
ros2 control list_hardware_interfaces        # 应看到 4 轮 + 6 关节的命令/状态接口
ros2 control list_controllers                # joint_state_broadcaster / mecanum_drive_controller / arm_controller
```

**默认是锁死的**（沿用现有安全设计）：

- 硬件接口默认**不打开串口**，且 DISARMED；需显式参数才连设备。
- 三个方向的速度限值**必须都是有限正数**才允许 ARM。`0.0` **不是**"不限速/透传"。
- ⚠️ **轮级等比饱和由 MCU 负责**（已核实 `mecanum_drive_controller` 只做机体轴级限幅，不做轮级）。
  MCU 收到 4 个轮速目标后，若任一超限则**四个同乘一个系数**，不得逐轴截断 —— 否则斜走变直走。
  控制器上的 `linear.x/y`、`angular.z` 限幅只作第一道防线，**不保证方向**。

### 6.1 物理层两个反直觉事实（2026-10-05 研究结论）

1. **USB CDC 的波特率是无意义的。** CDC-ACM 走 USB 批量端点，实际速率由 USB 决定
   （Full Speed 12 Mbps / High Speed 480 Mbps）；`SET_LINE_CODING` 里的 `dwDTERate`
   被典型 STM32 CDC 固件**忽略**。所以"921600"在 USB 链路上只是个约定值，
   固件里仍按它配置是为了**万一链路被桥接成真 UART**。设计时按**包传输**思考，不按波特率。
2. **⚠️ 必须关掉 USB 串口的 `latency_timer`。** 主机侧 FTDI/USB-serial 的
   `latency_timer` **默认 16 ms**，会把每一帧都攒 16 ms 再交付 —— 对 100 Hz 控制环
   是直接的抖动源。用 udev 规则设成 1 ms：

   ```bash
   # /etc/udev/rules.d/99-wheel-motor.rules
   ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
   # 稳定别名（不要用 /dev/ttyUSB0，雷达适配器也可能占）
   SUBSYSTEM=="tty", ATTRS{serial}=="<F407 的 USB 序列号>", SYMLINK+="wheel_motor", MODE="0660", GROUP="dialout"
   ```
   用 `lsusb -v` 或 `udevadm info -a -n /dev/ttyACM0` 取 F407 的真实序列号填进去。

> 注：下位机实测"200 ms lease 才稳"与本条相关 —— 16 ms 的 `latency_timer` 会放大
> 端到端抖动。**先修 latency_timer，再复测 50 ms lease 是否可行**，不要急着签字放宽到 200 ms。

上车顺序**严格按** [`hardware_acceptance.md`](hardware_acceptance.md)：PTY 回归 → 只读握手 →
拔线/超时/E-stop 撤销 → 轮子离地单轴低速 → 软件暂停与物理急停分开测 → 最后围栏地面测试。

**臂与轮现在在同一块板上**，但仍应**分开验收**：先只验轮（`joint_state_broadcaster` +
`mecanum_drive_controller`），确认 4 轮方向/符号/里程计正确；**再单独验舵机**
（关节名、零位、方向、限位、PWM 脉宽范围都要与固件对齐后才通电）。
不要一次把 4 轮 + 6 舵机全部 ARM。

---

## 7. 启动入口

```bash
ros2 launch rak_navigation slam.launch.py [odom_source:=rf2o|ekf|none]
ros2 launch rak_navigation nav2.launch.py                 # 含 lidar + SLAM + motion stub
ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim   # 默认 sim
ros2 run rak_task_executor task_cli                        # "go to <别名>"
ros2 launch rak_moveit_config demo.launch.py               # 旧五轴 demo
ros2 launch rak_moveit_config six_axis_sim.launch.py       # 六轴仿真夹具
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
```

`odom_source` 是里程计接入的**接缝**：现在是 `rf2o`（激光里程计），
换 `ekf` 后由 `robot_localization` 提供 `odom→base_link`（见 ADR-0002）。

**不要同时跑多个 Nav2 或多个 `/cmd_vel` 末端适配器。**

---

## 8. 部署检查单

- [ ] `colcon build` 通过（17 包），`check_workspace.py --installed` 通过
- [ ] `brain/tests/test_ai.py` 通过（`env -u PYTHONPATH`）
- [ ] 传感器单独跑通：`/scan` 有数据且 frame 是 `laser_link`
- [ ] `rak-mode.sh headless` 后双目不再 `VIDIOC_REQBUFS` 失败
- [ ] IMU：`spi0.0 -> st_lsm6dsx_spi`，`/proc/interrupts` 有中断计数
- [ ] 底盘：PTY 回归通过；真实串口**只读**握手能读到板 ID / 固件版本 / boot ID
- [ ] 三个速度限值已实测填入（非 0），ARM 前确认轮子离地
- [ ] 物理急停独立于 Jetson/ROS/网络验证过
- [ ] `docs/hardware_acceptance.md` 的 §Before Connecting a Physical Base 逐条签核

---

## 9. 常见误解

| 误解 | 事实 |
|---|---|
| "Jetson 跑 RT 内核才能控制电机" | 闭环在 MCU，Jetson 不需要 RT；装了通用 RT 内核反而起不来 |
| "麦轮 IK 应该在 MCU 上" | **已反转**（ADR-0007）：IK 在上位机 `mecanum_drive_controller`，MCU 只做每轮 PID + 舵机 PWM |
| "launch 能解析就等于硬件能跑" | 只证明静态 wiring；`hardware_enable` 默认 false |
| "有 swap 了就能随便开模型" | swap 防 OOM，不提供实时性；S1 的 158 ms / S2 的 12–30 s 不受影响 |
| "collision monitor 能代替急停" | 不能。物理急停与 MCU 看门狗必须独立验证 |
