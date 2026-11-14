# 机器基线（Jetson Orin Nano · 机器人核心）

> 状态：现行 · 最后更新：2026-11-12

> 更新日期：2026-11-13
> 适用：本仓唯一的工作区 `~/rak` 所在的那台 Orin Nano（`orin` / `orin.xrak`）。
> 一条命令回到已知状态：`sudo scripts/system/rak-system-baseline.sh`

## 这份文档回答什么

"这台机器**长什么样**"——不是"代码怎么写的"（那是 `docs/explanation/architecture.md`），
也不是"做到哪一步了"（那是 `docs/status/project_status.md`）。

刷机、换板、把机器借给别人之后再拿回来，都靠这份文档把状态拉回已知基线。
**凡是能脚本化的，都在 `scripts/system/rak-system-baseline.sh` 里；本文只解释为什么。**

## 一、两条互相打架的身份

这台机器同时是**开发机**和**机器人本体**。两者对系统配置的要求直接冲突：

| | 开发机要 | 机器人核心要 | 基线的取舍 |
|---|---|---|---|
| 图形 | GNOME 桌面、RViz、MoveIt | 不要桌面（CMA 全给相机） | **默认无头**，需要时 `sudo scripts/rak-mode.sh desktop` |
| 登录 | 人在键盘前 | 只允许密钥、开机即服务 | **禁用密码 SSH**，root 禁登 |
| 频率 | 按需升频省电 | 恒定最高频、抖动可预测 | **`jetson_clocks` 锁频** + 10W 满档 |
| 更新 | 自动打补丁 | 绝不允许半夜自动重启 | **关闭 unattended-upgrades** |
| 日志 | 越多越好 | 不能写满 NVMe | **journald 封顶 256 MiB** |

取舍原则只有一条：**机器人的确定性优先，开发的便利靠"需要时显式切换"补回来。**

## 二、SSH：只允许密钥登录

### 事实

| 项 | 值 | 来源 |
|---|---|---|
| `PasswordAuthentication` | `no` | `sshd -T` 实测 |
| `AuthenticationMethods` | `publickey` | 同上 |
| `PermitRootLogin` | `no` | 同上 |
| `AllowUsers` | `xrak` | 同上 |
| `MaxAuthTries` | `3` | 同上 |
| `authorized_keys` 条目 | 2（均为 ed25519） | `~/.ssh/authorized_keys` |

两把钥匙：

| 文件 | 指纹（公钥 base64 前 24 字符） | 备注 |
|---|---|---|
| `rak.pub` | `AAAAC3NzaC1lZDI1NTE5AAAAIEVKl5Ko` | 团队成员 |
| `xrak.pub` | `AAAAC3NzaC1lZDI1NTE5AAAAIM/5BOKOPcb+` | 主力工作站 |

### 为什么配置写在 `00-rak-hardening.conf`

JetPack 镜像的 `/etc/ssh/sshd_config.d/50-cloud-init.conf` 里有
`PasswordAuthentication yes`。sshd 对同一选项**取第一次出现的值**，而
`sshd_config` 顶部的 `Include /etc/ssh/sshd_config.d/*.conf` 按**文件名字典序**
加载。所以只要把我们的文件命名为 `00-`，它就一定排在 cloud-init 之前生效。

基线脚本做两件事，而不是一件：

1. 写 `00-rak-hardening.conf`（正本）；
2. 顺手把 `50-cloud-init.conf` 里的 `yes` 改成 `no`（防呆）。

第 2 步看起来冗余，但它防的是一种真实故障：有人为了排障临时调整 `Include`
顺序，或换了个镜像但复用了这份 `00-` 文件——那时 cloud-init 那份就会重新
成为"最后写入者"，密码登录**静默恢复**。

### 验证（不要只信配置文件）

```bash
sudo sshd -T | grep -i passwordauthentication     # 期望 no
ssh -o PubkeyAuthentication=no -o PreferredAuthentications=password xrak@127.0.0.1 true
# 期望: Permission denied (publickey).
```

### 旧 RSA 密钥已移除

镜像初始的 `authorized_keys` 里还有一把 `ssh-rsa ... xrak@Shizurak`（3072-bit RSA）。
基线只保留两把 ed25519，旧文件备份在 `~/.ssh/authorized_keys.bak-20261113`。
移除的理由不是"RSA 不安全"，而是**减少一把能进门的钥匙**——密钥数量应当等于
实际授权的人数。

### `kali` 不是 SSH 口令

`kali` 是 sudo 口令和 VNC 口令（团队默认值），**登不进 SSH**。这一点由
`AuthenticationMethods publickey` 强制。加人靠加公钥，不靠改配置。

## 三、性能：锁频与功耗档位

### 现状

| 项 | 值 | 怎么来的 |
|---|---|---|
| NV 功耗档位 | 10W（`ID=0`，Orin Nano DevKit 的最高档） | `nvpmodel.conf` 里只有 10W / 7W_AI / 7W_CPU 三档 |
| CPU governor | `schedutil`，但 **min = max = 1510 MHz** | `jetson_clocks`（无参数）把频率钉死 |
| GPU | 锁定 624.75 MHz | 同上 |
| EMC | 锁定 2133 MHz | 同上 |
| 服务 | `rak-clocks.service`（开机 oneshot） | 本仓新增 |

### 为什么 min = max 比 governor=performance 更好用

Tegra 的 cpufreq 只有 `schedutil`/`powersave`/`performance` 三种，而
`performance` 在某些 L4T 版本上会被 `power-profiles-daemon` 改回
`schedutil`。把 `scaling_min_freq` 顶到 `scaling_max_freq` 是**更硬**的约束：
无论 governor 是谁，频率都下不来。所以基线同时做两件事：

1. `jetson_clocks` 锁频；
2. **停用 `power-profiles-daemon`**（否则它会覆盖 governor 和部分频率设置）。

`/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor` 读出来仍是 `schedutil`
是**正常的**，别据此判断锁频失败——要看 `scaling_min_freq == scaling_max_freq`。

### `nvpmodel.service` 开机失败是已知的，已修

JetPack 自带的 `/etc/systemd/nvpmodel.sh` 在"当前档位已等于目标档位"时仍会问
`DO YOU WANT TO REBOOT NOW? enter YES/yes`，systemd 下没有 stdin，于是
`NVPM ERROR: bad input!` → `status=234` → 开机一条红字。

基线用一个 `drop-in` 把它换成 `/usr/local/sbin/rak-nvpmodel.sh`：
**已经在目标档位就什么都不做**，需要切换时才调 `nvpmodel -m`。
目标档位可用环境变量 `RAK_NVPMODE` 覆盖。

### 性能数字的可比性

`jetson_clocks` 锁频之后，历史基准（~158 ms/次 S1 推理、30 fps 双目）**不能
直接对比**——那些数字是在别的功耗/频率状态下测的。要比较就重新测，别拿旧数当基线。

## 四、内存：CMA 是相机能不能出图的前提

### 事实

| 状态 | MemAvailable | CmaFree（256 MiB 池） |
|---|---|---|
| 登录态 GNOME 桌面 | ~1.8 GiB | **~1.3 MiB**（曾测到 0） |
| 基线（无头） | ~2.4 GiB | **~49 MiB** |

`rak_stereo_driver` 用 `V4L2_MEMORY_MMAP` 申请 4 个缓冲，videobuf2 从 CMA 池里切。
桌面把池子吃干后，`VIDIOC_REQBUFS` 失败——**现象像相机坏了，实际是内存决策**。

### 一个容易误判的点：CmaFree 低 ≠ 相机一定采不到

2026-11-13 实测（本机，无桌面、无相机进程）：

| 时刻 | CmaFree |
|---|---|
| 稳态（跑了半天脚本/编译/文档） | **6 MB** |
| `sync; echo 3 > /proc/sys/vm/drop_caches` 之后 | **140 MB** |
| 之后 30 s（没有任何新负载） | 140 MB |

结论：CMA 池里的大部分**被页缓存借用**了。CMA 区域允许可移动分配（page cache、
匿名页）临时占用，真来一个 `cma_alloc`（例如相机申请 V4L2 MMAP 缓冲）时，
内核会把那些页迁走再腾出空间 —— 这是设计行为，不是泄漏。

区分"良性借用"和"真占用"，只需一条命令：

```bash
sync; sudo sh -c 'echo 3 > /proc/sys/vm/drop_caches'
awk '/CmaFree/{print $2/1024 " MiB"}' /proc/meminfo
```

- **掉完缓存后大幅回升**（像上面的 6 → 140 MB）→ 良性，相机能申请到缓冲。
- **掉完缓存后仍然很低** → 真有进程持着不可迁移的 DMA 缓冲。这时才去看
  `sudo ls /sys/kernel/debug/nvmap/handles_by_pid`、`dmesg | grep -i cma`，
  以及"是不是又有人把桌面开起来了"。

`scripts/rak-mode.sh status` 的告警已按这个判据改写：它现在会先报原始值，
再告诉你"低"是不是需要紧张的那种低。**不要看到 CmaFree 只有几 MB 就断定相机坏了。**

### 所以基线默认无头

`systemctl mask gdm3 gdm`：mask 而不是 disable，因为 `gdm.service` 是静态单元，
`disable` 是空操作；真正把显示管理器拉起来的是
`display-manager.service → gdm3.service` 这条别名。

需要屏幕时：`sudo scripts/rak-mode.sh desktop`（**自动登录是关的**，要在触摸屏输一次口令）。

### swap 与 zram

镜像自带 zram（2 GiB，zstd）和 `/swapfile`（2 GiB，`vm.swappiness=20`）。
"内存不够"在这台机器上不会立刻致命，但 **swap/zram 不构成实时保证**：
一旦换出，p99 延迟就没有上限。所以：**实时节点宁可 OOM 也不要换页**，
S1/S2 的延迟数字必须在"没换出"的前提下测量。

## 五、服务去冗余：停了什么，为什么

原则：**机器人不需要的桌面/云/调试服务停用，开发要用的留着**。
停用（`disable --now`）而不是卸载，是为了可逆——需要时
`systemctl enable --now <unit>` 就回来。

### 明确保留（不要停）

| 服务 | 为什么留 |
|---|---|
| `docker` / `docker.socket` / `containerd` | 开发环境靠容器（本机已有 `nvidia/cuda:12.6.3-base-ubuntu24.04`）。停掉等于砍掉一个开发手段 |
| `bluetooth` | 蓝牙手柄 / BLE 调试外设 |
| `avahi-daemon` | mDNS：开发期靠 `<host>.local` 互访，停掉后"名字解析不到"会被误当成网络故障 |
| `netclient` | Netmaker mesh，唯一外网入口（见下） |

前三类已进脚本的 `PROTECTED_UNITS`：写回禁用清单会被直接拒绝执行。

### 停用清单

| 服务 | 为什么停 |
|---|---|
| `lpd` / `cups` | 网络打印 |
| `gnome-remote-desktop` / `vino` | 与 `x11vnc` 抢 5900；README 实测 vino 重传 99.7% 帧缓冲 |
| `upower` / `udisks2` / `colord` / `accounts-daemon` / `switcheroo-control` | 纯桌面设施（需要桌面时 D-Bus 会按需激活，不必手工 enable） |
| `power-profiles-daemon` | 会把 governor 改回 `schedutil`，与锁频打架 |
| `sssd` | LDAP 客户端，机器人不在域里 |
| `openvpn` | 组网用 netmaker |
| `rsync` / `iperf3` / `lttng-sessiond` | 按需手动起，不必常开 |
| `nv-power-adapter-check` / `nv-tee-supplicant` | 厂商附带，无人使用 |

> **`netclient`（Netmaker）绝对不在停用之列。** 它是这台机器**唯一的外网入口**：
> mesh 地址 `100.121.213.x` 承载 DSH Web 转发、VNC（`100.121.213.20:5900`）与 SSH。
> 停掉它 = 远程失联，而机器人在现场时没有键盘可插。
> 同理不动：`NetworkManager`、`wpa_supplicant`、`systemd-networkd`、`systemd-resolved`。
> 2026-11-13 基线脚本一度把它误列为冗余，当场被拦下并从清单删除；脚本里现在有
> `PROTECTED_UNITS` 硬保护 —— 把命脉单元写进禁用清单会直接报错退出。
| `apport` | 崩溃上报往 `/var/crash` 落盘 |
| `haveged` | 内核 6.8 自带 jitter entropy，haveged 反而拖慢启动 |
| `rsyslog` | 日志已由 journald 承担，重复写盘 |
| `cloud-init*`（4 个单元） | 机器不联网初始化；而且它会**把 SSH 密码登录重新打开** |

> **保留**的 NVIDIA 单元（`nvargus-daemon`、`nvfancontrol`、`nvfb-*`、
> `nv-load-gpu-libs` 等）都是相机/风扇/GPU 库加载路径的一部分，**不要动**。

### 需要 GUI 时怎么办

停掉的服务里，`colord`/`upower`/`accounts-daemon` 在 GNOME 会话里会被
D-Bus 按需激活，所以 `rak-mode.sh desktop` 之后桌面照样能用——不需要手工
把它们 `enable` 回来。这是有意的：**默认精简，但不破坏"临时要桌面"这条路。**

## 六、日志、更新与存储

| 项 | 值 | 理由 |
|---|---|---|
| journald `SystemMaxUse` | 256 MiB | 机器人常年运行，不封顶会写满 NVMe |
| journald `MaxRetentionSec` | 2 week | 两周足够定位问题 |
| journald `RateLimitBurst` | 20000 | 默认 10000 条会静默丢 ROS 日志（"Suppressed N messages"） |
| `unattended-upgrades` | **关闭** | 自动升级内核会要求重启——对机器人不可接受，改人工排期 |
| `fstrim.timer` | 启用 | NVMe 健康 |

## 七、实时约束

`/etc/security/limits.d/99-rak-rt.conf`：

```
xrak - rtprio 95          # SCHED_FIFO 可用（控制环、S1 推理线程）
xrak - memlock unlimited  # CUDA pinned host 内存（PinnedBuffer）超过默认 64 MiB 会失败
xrak - nice -20
xrak - core unlimited
```

**登录后生效**：`limits.d` 由 PAM 在会话建立时读取，已登录的 shell 不会变。
验证：

```bash
ulimit -r    # 期望 >= 95
ulimit -l    # 期望 unlimited
```

## 八、机器人运行时接线

| 项 | 落点 | 说明 |
|---|---|---|
| CAN | `/etc/systemd/system/rak-can0.service` | 由 `scripts/can0-up.service` 改名安装；**含 Orin CAN pinmux 写入**（AON pinmux 重启必丢，漏写时 can0 照常 UP 但一帧收不到，且无任何报错） |
| 雷达别名 | `/etc/udev/rules.d/99-wheeltec-lidar.rules` | `/dev/wheeltec_lidar`；缺失时驱动报 `open_port ERROR`，像硬件故障 |
| VNC | `x11vnc.service` + `/usr/local/sbin/x11vnc-share` | 共享真实 X display；headless 下自动空转等待 |
| VNC 口令 | `/etc/x11vnc.pass` | 团队默认值（与 sudo 相同）；**不是 SSH 口令** |

### IMU 的 SPI pinmux 是单独的、需要重启的一步

`scripts/lsm6dsr-overlay/install-spi-pinmux.sh` 会把 `spi1` 四脚 pinmux 合进
设备树并改 `extlinux.conf`，**需要重启**。基线脚本**不**碰它：它改的是
bootloader 配置，属于"要么做要么不做"的一次性动作，不适合放进幂等基线。
另外，现场排查（2026-10-26）发现 SPI 路径卡在 CS 上电电平，**当前生产路径是 I2C**
（`rak_imu_driver` 的 `i2c_imu.py`）。SPI 快路径（`spi_imu.py` @1000 Hz）保留但未启用。

## 九、刷机后的恢复顺序

```bash
# 1. 基线（SSH / 功耗 / 无头 / 服务 / 日志 / CAN / udev / VNC）
sudo scripts/system/rak-system-baseline.sh

# 2. ROS 依赖（rosdep 会读 package.xml；本仓自定义键见 rosdep/rak-extra.yaml）
sudo rosdep init && rosdep update
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y

# 3. 工作区
python3 scripts/build_workspace.py --profile jetson

# 4. 重启（让 CAN pinmux / udev / limits 全部生效）
sudo reboot
```

第 4 步不是可选的：CAN pinmux 和 `limits.d` 都需要新会话/新开机才生效。

## 十、体检

```bash
sudo scripts/system/rak-system-baseline.sh --check
```

输出逐项列出：`PasswordAuthentication`、`authorized_keys` 条目数、功耗档位、
CPU governor、`CmaFree`、gdm 状态。**任何一项和本文表格不一致，就是基线漂移。**

## 参考

- `scripts/system/rak-system-baseline.sh` —— 本文所有配置的唯一实现
- `scripts/rak-mode.sh` —— headless ↔ desktop 切换（CMA 的取舍开关）
- `README.md` §Remote desktop —— VNC 的三条路由与"为什么不是 vino"
- `docs/tutorials/construction.md` §2.2 —— 内存/swap/zram 的现场记录
- `CLAUDE.md` §Gotchas —— CMA 争用、`vino` vs `x11vnc` 的实测数据
