# rak

面向 Jetson Orin Nano / ROS 2 Jazzy 的分层具身智能小车工程。
目标是 Jev（本地 TensorRT 决策模型，接入待完成）快速选择、云端 LLM + Harness 理解任务，
通过 Nav2、MoveIt2 和 F407 控制底盘与机械臂，而不是让模型直接控制电机。

设计入口：[项目总体状态](docs/project_status.md) · [整体架构](docs/architecture.md) ·
[统一迁移说明](docs/migration.md) · [硬件验收门槛](docs/hardware_acceptance.md) ·
**当前施工单：[2026-10-30 明天继续](docs/NEXT-2026-10-30.md)**。
包重命名不代表硬件已工程化；下面明确区分现有功能和后续规划。

点云抓取开发入口：[离线 GraspNet、RViz 与隔离仿真](docs/grasp_perception.md) ·
[抓取验收记录](docs/grasp_validation.md)。

## 工程布局

自有 ROS 包采用 `rak_` 前缀，按职责而非拟人化名称划分。

| 包 / 路径 | 职责与当前状态 |
|---|---|
| `src/rak_interfaces` | 感知/决策消息，以及任务 Action、控制 Service、底盘遥测 |
| `src/rak_gpu` | CUDA / TensorRT / VPI 工具；JetPack 专用 |
| `src/rak_stereo_driver` | 双目采集；nvJPEG 解码后拷回主机发布 ROS 图像 |
| `src/rak_astra_driver` | Astra 深度相机适配，保留为 Gemini 的可替换后端 |
| `src/rak_pointcloud` | GPU 点云节点（`gpu_pointcloud_node`） |
| `src/rak_grasp_perception` | 同步 RGB-D 快照、离线 GraspNet 候选、RViz ROI 与隔离规划预览；GPU 和实物验收进行中 |
| `src/rak_description` | URDF、几何、惯量、可视化；当前仍是旧五轴演示模型 |
| `src/rak_task_executor` | 唯一导航任务状态机；文字、暂停、恢复、取消和仿真 Nav2；依赖 `rak_intent` |
| `src/rak_intent` | 共享的版本化文本→意图契约；task executor 与 speech 共同使用 |
| `src/rak_speech` | 离线语音识别（Vosk）→ 转写 + 意图发布；板端唤醒/方位桥（`vtn_node`）与离线 TTS 播报（`tts_node`）；不自行下发目标 |
| `src/rak_wbc` | 全身运动仲裁层（ADR-0011，骨架 + 桌面数值测试，硬件门未开） |
| `src/rak_chassis` | F407 传输 + `ros2_control` 插件：`Session` 抽象，唯一实现 `CanSession`（SocketCAN CAN v4 唯一传输，ADR-0016）；当前导出 4 轮接口 |
| `src/rak_commissioning` | 标定/调试入口：`chassis_key_test` + `chassis_test.launch.py`；不是生产运动接口 |
| `src/rak_arm_simulation` | 软件轨迹/夹爪 Action server；无物理仿真、无硬件接口 |
| `src/rak_moveit_config` | 旧演示配置与隔离的六轴仿真配置；没有实机轨迹执行链 |
| `src/rak_motion_stub` | 测试转发与速度估算；不是 CAN 驱动或安全控制器 |
| `src/rak_bringup` | 唯一组装层：全部 launch（雷达/相机/slam/nav2/chassis_bringup/agent）、参数、udev、DDS 配置和操作脚本（ADR-0013 吸收 sensor_bringup/navigation/chassis_bringup） |
| `src/rak_agent` | （原 `brain/`）Jev / LLM 决策节点；ROS 系统 Python 内运行，模块名 `rak_agent`；语料在仓库根 `data/episodes/` |
| `src/vendor` | 第三方雷达和 rf2o；只读，不随自有包重命名 |

2026-10-09 起所有可构建包统一在 `src/` 下（ADR-0013）；完整的新旧映射见
`docs/migration.md`。

### 机器目录布局（`~` 下该有什么）

`~` 不是工作区，只有三样东西有资格在那儿（其余是系统模板目录和各类 agent/IDE 缓存）：

| 路径 | 是什么 | 版本管理 |
|---|---|---|
| `~/rak` | **唯一 ROS 2 工作区**（本仓库）。所有 `colcon build`、所有 `source` 都只认它 | 本仓库 |
| `~/ws_thirdparty` | 第三方驱动 overlay 壳：`src/OrbbecSDK_ROS2`（上游 `v2.10.6`，**从未编译**）。相机实际走 apt 的 `ros-jazzy-orbbec-camera` 2.9.3，**不要 source 这个 ws**；真要换源码版时才在它里面 `colcon build`，靠 source 顺序覆盖 apt 同名包 | 上游 clone |
| `~/vehical_rak_chassis` | STM32 F407 固件工程，**不是 ROS ws**。CAN v4 协议真源，被 `scripts/can_link_smoke.py`、`docs/adr/0016` 引用，不能挪 | 另一仓库 |

仓库内部另有两个**不入库**的本地目录（2026-10-14 从 `~` 归位，`.gitignore` 的 `/tools/`、`/debug/`）：

| 路径 | 是什么 |
|---|---|
| `~/rak/tools` | 本地工具与第三方 SDK 副本：`camera/`（OrbbecSDK 1.10.18）、`graspnet/`（抓取验证产物）、`remote_check.sh`、`iwlwifi-patches/` |
| `~/rak/debug` | 硬件调试小件（`spi_loopback.c`、`i2c_meter.sh`、`spi_meter.sh`） |

两条容易踩的坑：

1. **`colcon build` 只能在 `~/rak` 根目录跑**（`--base-paths src`）。在 `~` 下跑会在家目录
   砸出 `~/build` `~/install` `~/log` 三个孤儿产物，还会把 `~/ws_thirdparty`、
   `~/vehical_rak_chassis` 里的包一起卷进 discovery —— 2026-10-10 就这么发生过一次，
   那三个目录已删除。家目录永远不该出现 colcon 产物。
2. **`~/rak/tools/camera` 不是可清理的垃圾**：`rak_astra_driver` 的 CMake 缓存和
   `astra_node` 的 `RUNPATH` 烧的是 `/home/xrak/rak/tools/camera/.../SDK`（`ldd` 可验证），
   删掉或再移动会让深度相机直接编译失败、运行时找不到 `libOrbbecSDK.so`。不传
   `-DOrbbecSDK_DIR` 时 CMake 的默认回退值是 `/opt/OrbbecSDK_v1.10.18/SDK`，两份 SDK 并存，
   改路径必须重编 `rak_astra_driver`（`build/` 与 `build/camera-debug/` 两棵树）。

## 当前闭环

```text
文字 CLI -> rak_task_executor -> NavigateToPose Action -> Nav2
                           ├-> 暂停：取消确认 + 新鲜静止遥测
                           └-> 恢复：重发 map 目标并重新规划
Nav2 -> MPPI Omni -> smoother -> collision monitor -> /cmd_vel -> sim/stub adapter
CAN v4 -> F407 底盘（SocketCAN：chassis_bringup -> mecanum_drive_controller -> rak_chassis -> can0）
六轴 MoveIt fixture -> 软件轨迹/夹爪 Action server（仅仿真测试）
```

任务闭环有本地确定性文字解析和可选 RAI 慢路径地点选择；RAI 只能从配置的
waypoint 白名单中选别名，不能输出坐标或运动命令。当前 `rak_agent` 的 Laya/Jev
System 1 并未被改造成任务规划器，Harness 也不直接控制执行器。仿真任务入口用
fake Nav2 和“运动估算”遥测，不代表真实里程计或底盘反馈。
组合入口 `task_navigation.launch.py` 的 `motion_backend` 只有 `sim`/`stub` 两个值，
默认 `sim`；真底盘经 `chassis_bringup.launch.py` → `mecanum_drive_controller` →
CAN（ADR-0016，`can_channel` 默认 `can0`，`allow_motion` 默认 false）。不要把
launch 可解析当作固件兼容或安全验收。

机械臂目标仍是六个定位关节加独立夹爪。六轴 URDF/MoveIt 配置是明确隔离的测试夹具，
旧五轴模型仍是历史 demo；真实 CAD、标定、固件映射与反馈未提供。

## 构建与验证

在仓库根目录、只加载 ROS 基础环境的新终端中执行。使用新的构建/安装子目录，
避免旧包的安装产物混入验证；不删除旧构建结果。

```bash
source /opt/ros/jazzy/setup.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 colcon build \
  --base-paths src brain --executor sequential \
  --build-base build/architecture --install-base install/architecture \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/architecture/local_setup.bash
python3 scripts/check_workspace.py --installed
colcon test --build-base build/architecture --install-base install/architecture \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
ROS_DOMAIN_ID=77 python3 -m pytest \
  src/rak_task_executor/test src/rak_arm_simulation/test
PYTHONPATH=brain python3 brain/tests/test_ai.py
PYTHONPATH=brain python3 brain/tests/test_task_intent.py
```

ROS 集成测试需先 source `install/architecture/local_setup.bash`，并使用独立的
`ROS_DOMAIN_ID`；测试仅连接 fake action server，不访问真实 CAN 链路。
机器人运行统一使用 Cyclone DDS 和 domain 47（`~/.bashrc`、`rak_bringup` 操作脚本
及 Cyclone 配置保持一致）；ROS 集成测试仍隔离在 domain 77/187，避免加入机器人运行图。
完整构建需要本机 JetPack SDK。只开发决策层可用
`--packages-up-to rak_agent`，无需构建 GPU 包。
首次准备决策包依赖时使用系统包管理：
`rosdep install --from-paths brain --ignore-src -r -y`。`agent_node` 是标准 ROS
console entry point，直接使用 ROS 系统 Python；stub 不需要模型或 PyTorch。
TensorRT 原生 decider 尚未接入，需先取得经验证的 ONNX engine 与 tokenizer 资产。
导出依赖、固定版本、TensorRT profile 草案和当前内存门槛见
[Laya TensorRT 导出准备](docs/laya_tensorrt.md)。

## 启动入口

```bash
ros2 launch rak_sensor_bringup lidar.launch.py
ros2 launch rak_sensor_bringup astra.launch.py
ros2 launch rak_sensor_bringup depth_camera.launch.py  # 默认 Gemini 335L
ros2 launch rak_sensor_bringup depth_camera.launch.py camera:=astra
ros2 launch rak_sensor_bringup gemini335l.launch.py serial_number:=YOUR_SERIAL
ros2 launch rak_sensor_bringup stereo.launch.py device:=/dev/video0
ros2 launch rak_navigation slam.launch.py
ros2 launch rak_navigation nav2.launch.py
ros2 launch rak_task_executor task_simulation.launch.py
ros2 run rak_task_executor task_cli
ros2 launch rak_bringup task_navigation.launch.py motion_backend:=sim
ros2 launch rak_bringup chassis_bringup.launch.py  # 真底盘：ros2_control -> CAN（allow_motion 默认 false）
ros2 launch rak_moveit_config demo.launch.py
ros2 launch rak_moveit_config six_axis_sim.launch.py
ros2 launch rak_description view_urdf.launch.py
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
ros2 launch rak_bringup speech.launch.py  # 阵列:采集→转写/意图 + 板端唤醒 + TTS 播报
ros2 launch rak_speech asr.launch.py source:=arecord  # 备用直采,不经 audio_common
```

深度相机选择入口默认启动 Gemini 335L；切换 `camera:=astra` 使用既有 Astra 驱动，
同一入口每次只启动一个深度相机。335L 使用 Orbbec 官方 SDK v2 ROS 2 包
`orbbec_camera` 的 `gemini_330_series.launch.py`，该上游包需单独安装并 source；
本仓不复制或维护厂商 SDK。统一 RGB-D 话题和标定/TF 注意事项见
[相机接入说明](docs/cameras.md)。

六麦克风阵列（讯飞 XFM-DP / R818 降噪板）走标准 audio_common 管线：`speech.launch.py` 用
`audio_capture` 直连 ALSA 采集 16 kHz 单声道发布 `/audio`（六路波束成形在模块内
完成，主机侧 UAC 只见处理后的单声道），`asr_node` 订阅后发布 `/speech/transcript`
与 `/speech/intent`。同一入口还包含 `vtn_node`（板端唤醒+声源方位 →
`/speech/wake`，经 adb 隧道读板上 vtn socket）和 `tts_node` + `audio_play`
（`/speech/say` 服务 → sherpa-onnx 离线合成 → USB 喇叭）。设备 id、板端协议、
QoS、解码/合成速度实测与验收命令见
[麦克风阵列实测记录](docs/microphone_array.md)。

`task_simulation.launch.py` 是无传感器的任务状态机 fixture；`task_navigation.launch.py`
组合真实 SLAM/Nav2 软件栈，默认底盘后端为仿真估算，且仍需要有效雷达/定位输入。
`task_cli` 支持“go to <已配置别名>”及“暂停/继续/取消”。配置 `slow_planner:=rai`
时还需明确传入 LLM 配置；默认 `local` 不依赖云服务。六轴启动项没有物理仿真器，
只提供 MoveIt 与动作接口测试。不要同时运行多个 Nav2 或多个 `/cmd_vel` 末端适配器。
硬件验收前先阅读[硬件接入门槛](docs/hardware_acceptance.md)。真底盘由
`chassis_bringup.launch.py`（CAN）单独启动，`allow_motion` 默认 false，须由硬件
负责人确认链路、固件协议和测得的限速后再显式打开。
操作脚本见 [rak_bringup/scripts](src/rak_bringup/scripts/README.md)。

## 上报与 IMU 的分工（易混淆）

- 固件 `DRIVE_STATE`（100 Hz）：4 轮累计位置/速度——**纯编码器**，不含 IMU。
- 固件**轮级稳定器**（`wheel_stabilize_targets`）：用**下位机 IMU** 的 yaw 角速度
  做旋转补偿，只作用于轮速命令域；IMU 就绪是 ARM 前提之一（缺 IMU 时 ARM 会被
  外部故障门挡住）。这就是"板载 IMU 收窄为轮级"的职责（ADR-0007）。
- **航向权威在 Jetson**：未来经 40-pin SPI 接 LSM6DSRTR，由 EKF 做融合
  （P0-5 / P1-1）。两者不冲突：下位机 IMU 管 100 Hz 轮级抗扰，Jetson IMU 管
  全局航向与里程融合。

## 性能与硬件约束

- 双目历史测量为 1280×720 SBS、30 fps；独立解码线程、双缓冲和固定曝光帧率策略保留。
- 图像当前发布的是主机内存 `sensor_msgs/Image`，不是已完成的跨节点 GPU 零拷贝管线。
- 旧 PyTorch/Laya 路径曾测得约 158 ms/次、占约 1.4 GB 统一内存；这是历史基线，
  不是当前推荐运行方式，也不是急停实时环。目标为待接入的 TensorRT 原生推理。
- CUDA 13.2、TensorRT、VPI `/opt/nvidia/vpi4`、OrbbecSDK 1.10.18 仍按本机路径构建。
- `guard()` 保留最终否决权；Jev 进程内直接调用；S2 不得扩大安全许可。
- `brain/rak_ai/types.py` 和 episode 的 `state_text` 不变；新任务协议另行版本化。
- CAN 为 classic CAN 2.0A 1 Mbps，禁止 `fd on`。协议以叶节点仓库为准，不能猜测六轴固件映射。
- 底盘链路唯一传输是 CAN v4（SocketCAN `can0`，ADR-0016）；链路冒烟用
  `python3 scripts/can_link_smoke.py can0`（10/10 通过为合格：BAT≈12500 mV、FAULT=0、
  BUSOFF=0），物理层查 `ip -details -statistics link show can0` 与 `dmesg | grep -i can`。
  USB v2 链路、串口协议文档（`docs/protocol/`）与直连串口工具已按 ADR-0016 删除
  （删除前快照：tag `pre-usb-removal-20261013`）。
- collision monitor/Nav2 配置与软件测试不等于安全认证或物理急停。
- 桌面与相机竞争 CMA；不要为跑可视化而默认开启桌面。下面保留本机已有运维记录。

## Remote desktop (VNC)

The touchscreen is shared, not re-created: `x11vnc` attaches to whatever X
display GDM is currently running (`:0` at the greeter, `:1` once someone logs
in) and streams that exact framebuffer. There is no `Xvfb`/`Xvnc`, no second
compositor and no extra window manager — the whole server costs **~12 MiB RSS**
and touches 0 CMA.

### Quick connect

| Route | Address | Needs |
|-------|---------|-------|
| Wired LAN | `jetson.xrak:5900` (`192.168.5.108`) | Ethernet cable attached |
| WiFi LAN | `orin.xrak:5900` (`192.168.5.215`) | WiFi joined |
| Public (netmaker) | `100.121.213.20:5900` | netmaker up |

All three are one server on one port — same desktop, same password, pick whichever
route reaches the board. A name that resolves but then drops the connection is a
link problem, not a VNC one.

Password: `kali` (the team default — same as sudo).

**Coming from the pre-flash docs** (2026-10-03, board was hostname `orin`):

| Pre-flash | Now |
|-----------|-----|
| `orin`, `192.168.5.215` | the same board's **WiFi** route, now named `orin.xrak` |
| `100.121.213.15` | dead — netmaker re-addressed to `100.121.213.20` |
| (no wired name) | `jetson.xrak` / `192.168.5.108`, wired only |
| `x11vnc` | still `x11vnc` — it was never the thing that changed |

Any stock RFB client works, because this is plain VNC and not a vendor
protocol: macOS **Screen Sharing** (`vnc://orin.xrak:5900`), Linux
(`remmina`, `tigervnc-viewer`, `virt-viewer`), Windows (RealVNC / TightVNC /
UltraVNC, or Remmina). `-ncache 0` is set on purpose — client-side caching
makes macOS Screen Sharing and several Windows viewers render the desktop wrong.

### SSH on the same three routes

Port 22 behind the same names — drop `:5900`:

```bash
ssh xrak@jetson.xrak        # wired
ssh xrak@orin.xrak          # WiFi
ssh xrak@100.121.213.20     # netmaker
```

**Key-only.** `PasswordAuthentication no` / `PubkeyAuthentication yes`
(`sshd_config.d`), and `~/.ssh/authorized_keys` holds two ed25519 keys — so
`kali` will *not* get you an SSH shell even though it is the sudo and VNC
password. Nothing in this repo re-enables password SSH; adding a key is the way
in, not a config change.

### Why `x11vnc` and not `vino` (L4T's `README-vnc.txt` answer)

NVIDIA's `README-vnc.txt` ships `vino-server`, and `vino` was what this board ran
first. It was replaced on 2026-10-10 after measurement, not preference: **vino
ignores X DAMAGE inside the requested region**, so an incremental full-screen
request comes back re-encoded from the whole framebuffer. Against a *static*
desktop, one `incremental=1` full-screen request:

| Server | rects | pixels sent | share of framebuffer |
|--------|-------|-------------|----------------------|
| `vino` 3.22.0 | 19 | 612,777 | **99.7 %** |
| `x11vnc` | 5 | 1,736 | **0.3 %** |

Ten seconds of the same idle screen, two servers, same client traffic:

| Server | CPU | wire rate |
|--------|-----|-----------|
| `vino` | **96.4 %** | 2314 KiB/s (a real MacBook saw **19.8 Mbit/s**) |
| `x11vnc` | 1.6 % | 42 KiB/s |

So `vino` saturated a core to ship a screen that had not changed. 3.22.0-6ubuntu5
is noble's newest, so there is no upstream fix to wait for. The running service
now measures **0.1 % CPU over 10 s** with no client connected. The desktop itself
was ruled out as the cause — `gnome-shell` 6.5 %, `Xorg` 6.1 %, TCP clean
(`retrans 0/1012`) — and `-xdamage` was confirmed safe under GNOME Shell's
compositor (29.2 updates/s, 8.9 % CPU on a moving screen) despite x11vnc's own
warning about it.

`vino` is not uninstalled, just stopped and its autostart renamed to
`~/.config/autostart/vino-server.desktop.disabled` — it carries
`X-GNOME-AutoRestart=true`, so killing it alone would let it revive and fight for
port 5900.

### VNC vs. the camera — read before you log in

Sharing the *logged-in* session is exactly what starves `eye`, and that trade is
already recorded in `scripts/rak-mode.sh`: a logged-in GNOME session drives
`CmaFree` to **0 kB** of the 256 MiB pool (measured again during this setup:
800 kB), and `eye` needs that pool for 4 `V4L2_MEMORY_MMAP` buffers, so capture
fails at `VIDIOC_REQBUFS`. **x11vnc is not part of that cost** — it holds no CMA
and no GPU memory; the desktop is what costs. So:

- Camera / SLAM / nav2 work → `sudo scripts/rak-mode.sh headless`. gdm gets
  masked, the X server goes away, and x11vnc just idles (one log line per 10 s,
  ~0 memory) refusing connections until the display comes back.
- Need the screen → `sudo scripts/rak-mode.sh desktop`, type the password once
  at the touchscreen (autologin is deliberately off), log in, and VNC resumes on
  its own against the new `:1`.

So "VNC is down" after a `headless` is the expected state, not a fault.

### If the touchscreen goes dark

Most often it is not a fault: the board rebooted (or gdm restarted), nobody has
logged in yet — autologin is deliberately off — and the greeter idle-blanks after
`org.gnome.desktop.session idle-delay` (**300 s** for the `gdm` user, vs 900 s for
a real session). Touching the panel and typing the password brings it back; the
reboot itself is the thing worth explaining, not the darkness.

**Which state signals to trust — measured 2026-10-10 with the panel confirmed lit
by eye:**

| Signal | Reading while the screen was on | Trust it? |
|--------|--------------------------------|-----------|
| `/sys/class/drm/card2-DP-1/{dpms,enabled}` | `Off` / `disabled` | **No** |
| `/sys/kernel/debug/dri/2/state` CRTCs | `active=0` | **No** |
| `xrandr` | `DP-1 connected primary 1024x600+0+0  60.06*` | yes |
| `xset q` | `Monitor is On` | yes |
| `xwd -root` brightness | mean ≈ 131, 66 % non-black | yes |

`card2` is `nv_platform (13800000.display)` — the *tegra* DRM. The NVIDIA
proprietary driver drives the panel through its own path, so tegra's CRTCs stay
`active=0` **by design**, and its connector attributes never follow the real
panel. Diagnosing a dark screen from those files sends you in a circle (it cost a
lot of this session's time); use `xrandr` / `xset` / a framebuffer capture, or
just look at the panel.

Also expect X to move: `:0` is the greeter on vt1, `:1` the logged-in session on
vt2, each with its own `Xauthority`. `x11vnc` follows automatically (see the
wrapper below).

### Why a wrapper script

GDM does not keep one stable display: logging in or out tears down the X server
and starts a new one with a different number **and** a different
`Xauthority` (`/run/user/<uid>/gdm/Xauthority`). A service pointing at a fixed
`:0` therefore dies silently the first time someone logs in. `/usr/local/sbin/x11vnc-share`
re-derives both from the live `Xorg` process on every start; `x11vnc` exits when
its X server does, and systemd's `Restart=always` re-runs the probe. It also
copies the cookie to `/run/x11vnc.xauth` first, because `xauth` refuses to lock a
file owned by `gdm`/`xrak` even when run as root — that is what makes
`-auth guess` fail with `error in locking authority file`.

### Files and operations

The wrapper and its unit are version-controlled in `scripts/` — deploy them
onto a fresh board with:

```bash
sudo install -m 755 scripts/x11vnc-share    /usr/local/sbin/x11vnc-share
sudo install -m 644 scripts/x11vnc.service  /etc/systemd/system/x11vnc.service
sudo systemctl daemon-reload && sudo systemctl enable --now x11vnc
```

Runtime paths:

```bash
/usr/local/sbin/x11vnc-share        # wrapper: probe display + cookie, exec x11vnc
/etc/systemd/system/x11vnc.service  # Restart=always, After=display-manager (no Wants,
                                    #  so a masked gdm does not wedge it)
/etc/x11vnc.pass                   # VNC password, root 600

sudo systemctl status|restart x11vnc
journalctl -u x11vnc -f
sudo x11vnc -storepasswd <pw> /etc/x11vnc.pass   # to rotate the password
```

To check it without a GUI client, the handshake plus auth can be driven from the
command line (openssl's legacy provider does the VNC DES): the server should
answer `RFB 003.008`, offer security type `2`, accept `kali`, and return a
`1024x600` `ServerInit` with desktop name `jetson:1`.

## Hardware target

Jetson Orin Nano, JetPack 7.2 (CUDA 13.2, TensorRT 10.16, VPI 4.1.4).
