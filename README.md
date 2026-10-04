# rak

面向 Jetson Orin Nano / ROS 2 Jazzy 的分层具身智能小车工程。
目标是 Jev（本地 Laya）快速选择、云端 LLM + Harness 理解任务，
通过 Nav2、MoveIt2 和 F407 控制底盘与机械臂，而不是让模型直接控制电机。

设计入口：[整体架构](docs/architecture.md) · [统一迁移说明](docs/migration.md)。
包重命名不代表硬件已工程化；下面明确区分现有功能和后续规划。

## 工程布局

自有 ROS 包采用 `rak_` 前缀，按职责而非拟人化名称划分。

| 包 / 路径 | 职责与当前状态 |
|---|---|
| `src/rak_interfaces` | 感知与决策消息；无 GPU、模型依赖 |
| `src/rak_gpu` | CUDA / TensorRT / VPI 工具；JetPack 专用 |
| `src/rak_stereo_driver` | 双目采集；nvJPEG 解码后拷回主机发布 ROS 图像 |
| `src/rak_astra_driver` | 已测试的 Astra 深度相机接入；仍需标定与安装验证 |
| `src/rak_sensor_bringup` | 雷达、相机的设备配置与独立启动 |
| `src/rak_description` | URDF、几何、惯量、可视化；当前仍是旧五轴演示模型 |
| `src/rak_navigation` | 雷达 SLAM、里程计和 Nav2；当前不使用麦轮横移 |
| `src/rak_moveit_config` | SRDF、IK、规划配置；没有真实机械臂轨迹执行链路 |
| `src/rak_motion_stub` | 测试用 Twist 转发；不是 CAN 驱动或安全控制器 |
| `src/rak_bringup` | 整机集成入口、DDS 配置和操作脚本 |
| `brain/`（ROS 包 `rak_agent`） | Jev / LLM 决策原型；独立 Python 包仍为 `rak_ai` |
| `src/vendor` | 第三方雷达和 rf2o；只读，不随自有包重命名 |

`brain/` 保留在根目录是有意的：现有 `.venv`、模型依赖、episode 数据和
pip 工作流不随 ROS 包名迁移。`colcon` 同时发现 `src/` 与 `brain/`。

## 当前闭环

```text
雷达 -> 滤波 -> rf2o -> SLAM -> Nav2 -> /cmd_vel -> 测试转发 /cmd_vel_raw
相机 -> ROS 图像                         （尚无真实底盘通信）
模拟 / 外部 TrackedObjects -> rak_agent -> /brain/decision
旧机械臂模型 -> MoveIt 规划演示           （尚无真实轨迹执行）
```

这几条链路尚未形成统一任务闭环。目标硬件是六个定位关节加独立夹爪，
不能将旧模型的固定夹爪改成活动关节就宣称支持六轴机械臂。
任务执行、打断恢复、语音、感知融合和硬件安全接入见架构文档。

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
colcon test --build-base build/architecture \
  --packages-select rak_description rak_moveit_config
colcon test-result --test-result-base build/architecture --verbose
env -u PYTHONPATH brain/.venv/bin/python brain/tests/test_ai.py
```

完整构建需要本机 JetPack SDK。只开发决策层可用
`--packages-up-to rak_agent`，无需构建 GPU 包。
首次准备 Python 环境：在 `brain/` 下创建 `.venv` 并安装 `-e .`；
Laya / RAI 分别是可选的 `.[laya]` / `.[rai]`，stub 不依赖模型。
venv 命令需 `env -u PYTHONPATH`，ROS 节点则必须保留 ROS 的 `PYTHONPATH`。

## 启动入口

```bash
ros2 launch rak_sensor_bringup lidar.launch.py
ros2 launch rak_sensor_bringup astra.launch.py
ros2 launch rak_sensor_bringup stereo.launch.py device:=/dev/video0
ros2 launch rak_navigation slam.launch.py
ros2 launch rak_navigation nav2.launch.py
ros2 launch rak_moveit_config demo.launch.py
ros2 launch rak_description view_urdf.launch.py
ros2 launch rak_bringup agent.launch.py decider:=stub sim:=true
ros2 run rak_agent agent_node --ros-args -p sim:=true -p decider:=stub
```

以上是独立入口，不要全部同时启动：Nav2 已包含雷达与 SLAM；
机械臂 demo 的静态 `world -> base_link` 也不能直接与移动底盘 TF 合并。
实际执行会启动相应设备，硬件测试前先阅读架构文档中的安全缺口。
操作脚本见 [rak_bringup/scripts](src/rak_bringup/scripts/README.md)。

## 性能与硬件约束

- 双目历史测量为 1280×720 SBS、30 fps；独立解码线程、双缓冲和固定曝光帧率策略保留。
- 图像当前发布的是主机内存 `sensor_msgs/Image`，不是已完成的跨节点 GPU 零拷贝管线。
- Laya 历史测量约 158 ms/次、占约 1.4 GB 统一内存（10 W 模式）；不是急停实时环。
- CUDA 13.2、TensorRT、VPI `/opt/nvidia/vpi4`、OrbbecSDK 1.10.18 仍按本机路径构建。
- `guard()` 保留最终否决权；Jev 进程内直接调用；S2 不得扩大安全许可。
- `brain/rak_ai/types.py` 和 episode 的 `state_text` 不变；新任务协议另行版本化。
- CAN 为 classic CAN 2.0A 1 Mbps，禁止 `fd on`。协议以叶节点仓库为准，不能猜测六轴固件映射。
- 桌面与相机竞争 CMA；不要为跑可视化而默认开启桌面。下面保留本机已有运维记录。

## Remote desktop (VNC)

The touchscreen is shared, not re-created: `x11vnc` attaches to whatever X
display GDM is currently running (`:0` at the greeter, `:1` once someone logs
in) and streams that exact framebuffer. There is no `Xvfb`/`Xvnc`, no second
compositor and no extra window manager — the whole server costs **11.6 MiB RSS**
and touches 0 CMA.

| Network | Address |
|---------|---------|
| LAN | `orin.xrak:5900` (i.e. `192.168.5.215`, wired `192.168.5.108`) |
| Public (netmaker) | `100.121.213.15:5900` |

Password: `kali` (the team default — same as sudo).

Any stock RFB client works, because this is plain VNC and not a vendor
protocol: macOS **Screen Sharing** (`vnc://orin.xrak:5900`), Linux
(`remmina`, `tigervnc-viewer`, `virt-viewer`), Windows (RealVNC / TightVNC /
UltraVNC, or Remmina). `-ncache 0` is set on purpose — client-side caching
makes macOS Screen Sharing and several Windows viewers render the desktop wrong.

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
`1024x600` `ServerInit` with desktop name `xrak:1`.

## Hardware target

Jetson Orin Nano, JetPack 7.2 (CUDA 13.2, TensorRT 10.16, VPI 4.1.4).
