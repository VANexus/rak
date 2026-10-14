# 上位机（Jetson）Agent 交接文档

> 面向后续接手的 AI Agent / 开发者。记录 2026-10-09 全天的**环境真相、已完成状态、已知坑与待办**。
> **动手前先读完本文**，能省一整天的重复踩坑。姊妹文档：`docs/CONSTRUCTION.md`（构建规范）、
> `docs/architecture.md`（架构）、`docs/project_status.md`（路线图）。

---

## 0. 一分钟速览

- 本仓 = 机器人**上位机**（Jetson Orin Nano）。下位机 F407 固件在姊妹仓 `vehical_rak_chassis`（**已移交别人维护，勿改**）。
- 当前主任务：**四路里程计（IMU/轮式/雷达/SLAM）→ robot_localization EKF 融合 → Nav2 避障导航**。
- 2026-10-09 完成：IMU 全链路、SLAM+Nav2 首跑（闭环 0.4m→2cm）、EKF 融合（30Hz）、lifelong 持久地图——**全部实跑验证**。
- **最大陷阱（☆☆☆）：所有 ros2 操作必须先 source 团队环境**，否则默认 FastDDS+域0 与栈互相看不见：

```bash
source ~/rak/src/rak_bringup/scripts/environment.sh
```

---

### 0.1 2026-10-10 增量（当日会话）

- **CAN 链路软件侧就绪**：底盘 bringup 走 CAN v4（`transport`/`device` 参数与 USB 备用
  链路已按 ADR-0016 于 2026-10-14 删除）。空载冒烟已过（无 F407：CM+双控制器 active、CanSession
  在发帧、odometry 话题正常）。**底盘板尚未接**（can0 零流量；2026-10-13 已接板联调：
  H/L 对调后 `can_link_smoke.py can0` 10/10，见 ADR-0016）。
- **can0 开机持久化**：Orin CAN pinmux 写入并入 `scripts/can0-up.service`——此前重启后
  pinmux 回默认 0xC415，can0 照常 UP 但一帧收不到且无报错（人工 devmem 的时代结束）。
- **IMU 链路修复（I2C 时代，同日傍晚已退役）**：①芯片地址漂移 0x6a→0x6b，
  `lsm6dsr-i2c-setup.sh` 与 `iio_imu.py` 均自动探测两地址；②**不再创建内核 i2c 设备**——
  st_lsm6dsx 驱动绑定后会把 accel ODR（CTRL1_XL=0x10, 12.5Hz）**持续强写回**，覆盖 208Hz
  配置（实测绑定期间写 0x50 在 0.3s 内被回退；解绑后稳定保持）。数据路径本就是裸 i2c 块读，
  不需要 IIO 设备。实测 100.0Hz、|a|≈9.9 m/s²。**该 I2C 链当日傍晚整体删除退役
  （用户定案：只要 SPI）**——教训保留在 §2.1/§3.1。
- **IMU 切 SPI（进行中，当日午后）**：①根因——原厂 stock DTB 下 40 针 SPI1 五脚
  （PZ3-7）**MUX UNCLAIMED**（spidev 设备在、引脚无信号），且本平台无可靠运行时 pinmux
  写路径（debugfs pinmux-select 假成功、configfs dt-overlay 未编入内核）→ 必须启动期应用；
  ②已装 `scripts/lsm6dsr-overlay/rak-imu-spi-pinmux.dts`（pinctrl hog，同 jetson-io 的
  exp-header-pinmux 机制；属性值取自团队 rak_lsm6dsr.dtb）+ `install-spi-pinmux.sh`
  （fdtoverlay 预合并 → `/boot/dtb/rak_imu_spi.dtb`，extlinux primary FDT 已切、备份
  `extlinux.conf.pre-spi-*`，**重启生效**）；③节点 `scripts/spi_imu.py`（spidev0.0
  mode3@10MHz、1.66kHz ODR→1000Hz 发布、`--probe` 只读自检、无需 ROS 环境）。
  ④切换次序：模块接线 3/5(I2C)→23/19/21/24(SPI) → 重启（**pinmux 已验证生效**：
  四脚 HOG claimed by spi1，2026-10-10 15:59）→ `spi_imu.py --probe` → 起节点。
  **旧 I2C 链已删除退役**（用户定案"只要 SPI"）：iio_imu.py / lsm6dsr-i2c.service /
  lsm6dsr-i2c-setup.sh 三件套已删、/etc 单元已移除（需要时从 git 历史取回）。
  回滚 SPI = 还原 extlinux 备份 + 恢复接线（+ 如回 I2C 再从 git 恢复三件套）。
  注：LSM6DSR 无恰好 1000Hz 档位（…833/1660），用 1.66kHz 保证 1kHz 读取每拍都是新样本。
- **environment.sh 修复**：工作区探测 marker `brain/package.xml` 在 ADR-0013 重构后不复存在
  → 所有 run_*.sh 一 source 即退出（`ae6b31f`）；已改以 `src/rak_bringup/package.xml` 为 marker。
- **预提交门禁恢复并清干净**：b2594ed 修 setup.cfg 后门禁才开始真正执行，暴露三处历史漏并
  全部补齐（rak_bringup→nav2_lifecycle_manager/robot_localization；rak_moveit_config→
  rak_arm_pinocchio_ik），`check_workspace.py` 全绿。
- 栈状态：上午机器重启过，SLAM/Nav2 全栈未起（当前只有 IMU 节点 + 队友 IK 演示栈）。

---

## 1. 环境与连接

### 1.1 进入 Jetson

- `ssh xrak`（隧道主机 `orin.xrak`；也曾走局域网 `192.168.5.215`——网络会变，断了先试隧道）
- sudo 密码：`kali`
- **pkill 自匹配陷阱**：`pkill -f 'lslidar_driver_node'` 会连同你自己的 shell 一起杀掉（-f 匹配整条命令行）。
  一律用方括号脱敏：`pkill -f 'lsl[i]dar'`、`pkill -f 'nav2[.]launch'`、`pkill -f 'rf2[o]'`——或直接按 PID kill。

### 1.2 团队环境（最大坑，写三遍都不过分）

`environment.sh` 一次性设置四件事，缺一看不见栈：

| 项 | 值 | 说明 |
|---|---|---|
| RMW | `rmw_cyclonedds_cpp` | 不是默认 FastDDS |
| ROS_DOMAIN_ID | `47` | 不是 0 |
| 安装空间 | `install/architecture` | 非默认 install/ |
| CYCLONEDDS_URI | rak_bringup/config/cyclonedds.xml | |

### 1.3 构建（规范命令见 `docs/CONSTRUCTION.md` §3）

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build --base-paths src --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DOrbbecSDK_DIR=/home/xrak/rak/tools/camera/orbbec-sdk-1.10.18/OrbbecSDK_v1.10.18/SDK
```

- `-DOrbbecSDK_DIR`：astra 驱动需要（SDK 已在 `~/rak/tools/camera/` 里；缺它 build 停在 rak_astra_driver）
- `libpcap-dev` 需要 apt 装好（lslidar 编译要 pcap.h；运行时要 -lpcap）
- **pre-commit 钩子**：工作区有队友未跟踪在制品（如 `rak_arm_pinocchio_ik` 缺 setup.cfg）时钩子会崩。
  确认与己无关可以 `git commit --no-verify`，并在提交信息里注明原因。
- 只有本机（Jetson）能完整构建；改动前先在**源文件**（`src/`）编辑再重编，不用手改 install/。

### 1.4 启动 / 停止整套栈

```bash
cd ~/rak
bash src/rak_bringup/scripts/run_slam.sh                    # SLAM 链（rf2o 档默认）
bash src/rak_bringup/scripts/run_nav2.sh                    # 全栈 lidar+SLAM+Nav2+adapters
bash src/rak_bringup/scripts/run_nav2.sh odom_source:=ekf   # EKF 档（推荐）
```

- 日志：`/tmp/slam_run.log`、`/tmp/nav2_run.log`；脚本自带环境+setsid 后台
- 停止：方括号 pkill（§1.1）或按 PID
- ROS 节点清单自查：`ros2 node list`；生命周期：`ros2 lifecycle get /controller_server`

---

## 2. 已完成状态（2026-10-09 实跑验证）

### 2.1 IMU（LSM6DSRTR）—— I2C 链（已退役，2026-10-10）

> **现役链路 = SPI**（`scripts/spi_imu.py`，1000Hz；见 §0.1）。I2C 三件套已按用户定案
> 删除（git 历史可溯）；以下内容保留为历史记录与教训（地址漂移 / 内核驱动回写 bug）。

- **物理**：I2C 接 40 针（SCL→5 脚、SDA→3 脚、VCC→1、GND→6），芯片 @ `0x6a` on `i2c-7`
  （历史弯路：SPI 路线折腾一晚——接触不良+驱动问题；最终 I2C 首探即通，2026-10-09 定案）
- **systemd 单元** `scripts/lsm6dsr-i2c.service`（enabled）→ 调 `scripts/lsm6dsr-i2c-setup.sh`：
  地址探测（0x6a/0x6b）+ **裸寄存器配置**（`CTRL3_C=0x44, CTRL1_XL=0x50, CTRL2_G=0x50`）。
  **2026-10-10 起不再 `new_device`**：内核驱动绑定后会把自己的 12.5Hz accel ODR 强写回
  CTRL1_XL，覆盖 208Hz 配置（见 §3.1 追补）；数据路径=裸 i2c 块读，不需要 IIO 设备
  （iio scale 兜底值即正确量程）
- **ROS 节点**：`scripts/iio_imu.py`（ctypes I2C_RDWR 一次事务块读 0x22..0x2D 12 字节，
  100Hz → `/imu/data_raw`，frame=`imu_link`。**不走内核 sysfs 读数**——见 §3.1）
- 验证基线：`/imu/data_raw` @100.0Hz；静止 \|a\|≈1.01g、陀螺噪声 ±0.1°/s
- 启动（在团队环境里）：`nohup python3 scripts/iio_imu.py > /tmp/imu_node.log 2>&1 &`
- **迁移定案（2026-10-10 傍晚）**：I2C 链删除退役、SPI 唯一（切换详情见 §0.1）。

### 2.2 SLAM + lifelong 持久地图

- `slam_toolbox` 已切 **`mode: lifelong`**（`config/slam_toolbox.yaml`）：位姿图存盘
  `data/maps/rak_room.posegraph`（约 10MB 全轨迹），开机自动加载、地图跨会话无限延伸
- 读图/存图：

```bash
# 存盘（搬地方之前先跑）
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '/home/xrak/rak/data/maps/rak_room'}"
# 静态图导出（给 AMCL 备用）
ros2 run nav2_map_server map_saver_cli -f data/maps/room_$(date +%m%d)
```

- 已有静态图：`data/maps/room_20261009.pgm/.yaml`；`data/` 在 .gitignore（地图不入库）

### 2.3 Nav2 全栈

- 9 个节点（lslidar+scan_filter+rf2o+slam+6 服务器全 active，MPPI 控制器）
- **导航闭环实测**：目标 0.4m → 规划/控制/自动恢复 10 次 → `distance_remaining` 收敛 2cm
- cmd_vel 链：`controller → cmd_vel_nav → smoother → cmd_vel_smoothed → collision_monitor → cmd_vel → adapter`
  （adapter 有 sim/stub/none 三档，默认 sim；真轮接上后由 ros2_control 接 `/cmd_vel`）

### 2.4 EKF 融合（robot_localization）

- **`odom_source:=ekf` 分支已实现**（`launch/slam.launch.py`；团队注释未动，改动有 `[AI]` 标记）：
  - rf2o 在 ekf 档**仍运行但 `publish_tf=False`**（转为纯输入源；源码 `if(publish_tf)` 守卫已核对）
  - `ekf_filter_node`（`config/ekf.yaml`）接管 `odom→base_link` **唯一发布**
  - 新增 `base_link→imu_link` 静载 TF（安装姿态待装车标定）
- 输入：`odom0=/odom`(rf2o: x,y,yaw,vx,vy) + `imu0=/imu/data_raw`(仅 vyaw=航向速率权威)
- 输出：`/odometry/filtered` @30Hz；实测融合值跟踪 rf2o 原始值
- **轮式 odom1 槽位已在 ekf.yaml 注释预留**（就位即插；yaw/vyaw 按团队规矩排除——麦轮打滑）

### 2.5 TF 树（正确形态，勿破坏）

```
map ──slam_toolbox──► odom ──EKF──► base_link ──┬─ static: laser_link
                                                 └─ static: imu_link
```

---

## 3. 已知坑（勿重复踩）

1. **内核驱动 st_lsm6dsx 两宗罪**：①sysfs 写路径 desync——写 `sampling_frequency` 报成功但
   **寄存器不写**（解绑/重绑也不修）→ IMU 配置一律**裸 i2cset**；②**绑定后会把自己的
   accel ODR（12.5Hz）持续强写回 CTRL1_XL**，裸 i2cset 写的 208Hz 在 0.3s 内被覆盖
   （2026-10-10 实测；处置=根本不创建内核 i2c 设备，见 §2.1）。
   *（团队级 bug，值得上报内核社区/ST）*
2. **CTRL3_C 必须写 0x44**（BDU+IF_INC）。写 0x40 会清掉默认 IF_INC → 块读 12 字节全是同一寄存器
   （症状：陀螺/加计 x=y=z 三轴同值）。
3. **lslidar 堆 bug**：偶发 `malloc(): invalid size (unsorted)` 崩溃（exit -6，非确定性）。
   症状：/scan 断流、rf2o 一直 "Waiting for laser_scans"。
   处置：单独 `ros2 launch rak_bringup lidar.launch.py`（后台）拉起即自愈。
   **建议团队**给 launch 的 lslidar 节点加 `respawn=True`。
4. **`scripts/check_workspace.py --installed` 会崩**：import rak_agent 链触发 TensorRT engine 缺失
   （找 `/tmp/brain/data/models/...` 的 .plan）。静态模式（无 --installed）正常。
5. **pre-commit 钩子**：见 §1.3。钩子崩 = 环境/队友在制品问题，与你的提交相关性需自行判断，
   确认无关可 `--no-verify` 并注明。
6. **pkill 自匹配**：见 §1.1。这个坑会杀掉你自己的会话、且没有任何输出，极难察觉。
7. **SSH 网络易变**：局域网 IP 与隧道都会变；重连先试 `ssh xrak`，再 `ipconfig/nslookup`。
8. **雷达设备**：`/dev/serial/by-id/usb-Silicon_Labs_CP2102_...-0001-if00-port0`
   （udev 别名 `/dev/wheeltec_lidar` 未装；`lidar.launch.py` 有回退逻辑，正常）。
9. **杜邦线接触不良是本项目最大硬件坑**（IMU 折腾一晚上全因它）。动线后务必复测
   （IMU 用 `i2cget -f -y 7 0x6a 0x0f` 应回 0x6b）。
10. **IMU 地址可能变（I2C 时代教训，已退役）**：SA0 线态决定 `0x6a`/`0x6b`（2026-10-10
    实翻 0x6b，全链失联）。SPI 单链路后无此问题；回退 I2C 时见 git 历史里的自动探测实现。

---

## 4. 待办路线图（按优先级）

1. **轮式里程计（下一项；软件侧 2026-10-10 已就绪，等硬件）**
   - 接线：Jetson 收发器模块（CANH/CANL/GND）↔ A 板 CAN 端子；**H/L 别反、必须共地**
     （2026-10-09 联调根因，见 ADR-0012）；电池上电；**车轮架空**
   - 传输：**CAN v4 唯一链路**（ADR-0012/0016；`transport` 参数已删除，
     `chassis_bringup.launch.py` 的 `can_channel` 默认 `can0`）
   - 先跑 `python3 scripts/can_link_smoke.py can0`（协议级冒烟，全程不发非零速度指令）
   - 量四轮几何（轮径/轮距/轴距）→ 更新 `config/chassis_controllers.yaml`
     （现为占位值 32.5mm / 0.15m，**未签核**）
   - ros2_control 握手（`allow_motion:=false` 先跑）→ 架空点动 → 读
     `/mecanum_drive_controller/odometry` 验证
   - EKF 加 `odom1`（槽位在 ekf.yaml 里）
2. **IMU 装车姿态标定**：量出 base_link→imu_link 的真实旋转 → 更新 launch 静载 TF 与
   spi_imu 的 frame 约定
3. **spi_imu 正式化**：并入某个包 + 自启动（注意 check_workspace.py 的 21 包守卫，
   新增包要过团队）
4. **报团队三个 bug**：§3.1（驱动 desync）、§3.3（lslidar 堆崩溃）、§3.4（门禁崩溃）
5. **Nav2 真跑避障**：轮式就位后，用已存的 lifelong 地图 + EKF 里程源实跑

---

## 5. 关键文件地图

| 路径 | 作用 |
|---|---|
| `src/rak_bringup/scripts/environment.sh` | **团队环境（必 source）** |
| `src/rak_bringup/scripts/run_slam.sh / run_nav2.sh` | 启动入口（后台+日志) |
| `src/rak_bringup/launch/slam.launch.py` | SLAM 链 + `odom_source` 缝（rf2o/ekf/none） |
| `src/rak_bringup/config/ekf.yaml` | EKF 融合配置（轮式槽位预留） |
| `src/rak_bringup/config/slam_toolbox.yaml` | lifelong 持久地图配置 |
| `scripts/spi_imu.py` | IMU ROS 节点（SPI 块读 1000Hz；`--probe` 只读自检） |
| `scripts/lsm6dsr-overlay/rak-imu-spi-pinmux.dts` + `install-spi-pinmux.sh` | 40 针 SPI1 pinmux overlay（预合并进 /boot/dtb/rak_imu_spi.dtb） |
| `data/maps/rak_room.posegraph` | lifelong 位姿图（gitignore，本地资产） |
| `docs/CONSTRUCTION.md` | 构建规范（§3 命令） |
| `src/vendor/lslidar/.../CMakeLists.txt` | 已修（签名混用+ament 回正） |

## 6. 交接时刻的服务器快照（2026-10-10 傍晚）

- 在跑：无 IMU 节点（I2C 已删、SPI 待接线）；桌面 RViz/IK 演示随 15:57 重启结束
- can0：UP、1Mbps、pinmux 持久化 ✓；**零流量 = 底盘板未接**
- SPI：pinmux 已生效（四脚 HOG claimed by spi1）、`/dev/spidev0.0` 就绪；**模块未换线**
  （i2c-7 上 0x6b 仍在应答——线要"搬"不要"加"）
- 地图：`data/maps/rak_room.posegraph`（lifelong，未动）
- git：工作树有**队友会话未提交改动**（语音模块 rak_speech/文档/attic 删除等，勿动）；
  IMU 相关提交最新为 I2C 退役 commit
- 固件仓：R63（CAN v4；tags: hw-freeze-r43 / can-freeze-r52）