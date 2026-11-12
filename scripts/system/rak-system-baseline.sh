#!/usr/bin/env bash
# rak-system-baseline.sh -- 把这台 Orin Nano 变成"机器人核心"的机器级基线。
#
# 为什么有这个脚本：这台机器同时是**开发机**（要 ROS 2 / MoveIt / RViz / 编译）
# 和**机器人本体**（要 CAN / 相机 / 实时性 / 开机即用）。两种身份互相打架：
#
#   * 图形桌面吃掉 CMA —— 登录态 GNOME 会把 256 MiB 的 CMA 池压到 0 kB，
#     而双目相机要用 V4L2_MEMORY_MMAP 申请 4 个缓冲，采不到帧（表现为"相机坏了"）。
#   * cloud-init 在镜像里打开 PasswordAuthentication —— 密码登录等于把机器人
#     摆在公网上。
#   * 功耗档位没钉死 —— 板子在一堆模式下跑，基准数字没法互相比较。
#
# 所以"机器长什么样"必须固化成**可重复执行**的一条命令：刷机后跑一遍 == 回到已知基线。
#
# 用法:
#   sudo scripts/system/rak-system-baseline.sh            # 应用基线
#   sudo scripts/system/rak-system-baseline.sh --dry-run  # 只打印将要做什么
#   sudo scripts/system/rak-system-baseline.sh --check     # 只体检，不改动
#
# 幂等: 可重复执行。
# 不做的事: 不卸载任何 apt 包（只停用服务）、不动分区、不动 bootloader 配置、
#           不删用户数据、不碰 ROS 工作区内容。

set -euo pipefail

REPO="$(cd "$(dirname "$(readlink -f "$0")")/../.." && pwd)"
MODE=apply
case "${1:-}" in
  --dry-run) MODE=dry ;;
  --check)   MODE=check ;;
  "")        MODE=apply ;;
  *) echo "用法: $0 [--dry-run|--check]" >&2; exit 2 ;;
esac

[ "$(id -u)" -eq 0 ] || { echo "需要 root: sudo $0" >&2; exit 1; }

say()  { printf '\n== %s ==\n' "$*"; }
info() { printf '   %s\n' "$*"; }
warn() { printf '   !! %s\n' "$*" >&2; }

# 在 dry-run 下只打印；check 模式下只报告状态。
step_do() { # step_do <描述> <命令...>
  local desc="$1"; shift
  if [ "$MODE" = dry ]; then
    info "[dry-run] $desc"
    return 0
  fi
  if "$@" >/dev/null 2>&1; then
    info "ok: $desc"
  else
    warn "跳过（失败）: $desc"
  fi
}

# ---------------------------------------------------------------- 0. 前置检查
say "0. 前置检查"
for t in busybox ip systemctl jetson_clocks nvpmodel; do
  command -v "$t" >/dev/null 2>&1 || warn "缺少工具: $t"
done
info "仓库: $REPO"

# ------------------------------------------------------- 1. SSH 只允许公钥登录
say "1. SSH：只允许公钥登录"
if [ "$MODE" = check ]; then
  info "PasswordAuthentication = $(sshd -T 2>/dev/null | grep -i '^passwordauthentication' | awk '{print $2}')"
  info "authorized_keys 条目 = $(grep -c 'ssh-' /home/xrak/.ssh/authorized_keys 2>/dev/null || echo 0)"
else
  install -d -m 0755 /etc/ssh/sshd_config.d
  cat > /etc/ssh/sshd_config.d/00-rak-hardening.conf <<'EOF'
# rak robot core -- sshd hardening
# 目标：只允许公钥登录，禁止密码/交互式认证；只允许 xrak 账户。
# 放在 sshd_config.d/ 最前（00- 排序最先被 Include），利用 sshd "首次取值生效"
# 覆盖掉 50-cloud-init.conf 里的 PasswordAuthentication yes。
PasswordAuthentication no
PermitEmptyPasswords no
KbdInteractiveAuthentication no
PubkeyAuthentication yes
AuthenticationMethods publickey
PermitRootLogin no
AllowUsers xrak
MaxAuthTries 3
LoginGraceTime 30
AllowAgentForwarding no
EOF
  chmod 0644 /etc/ssh/sshd_config.d/00-rak-hardening.conf
  # cloud-init 那份也要就地修掉：万一以后有人调整 Include 顺序，它会再把密码登录打开
  sed -i 's/^PasswordAuthentication yes/PasswordAuthentication no/' \
    /etc/ssh/sshd_config.d/50-cloud-init.conf 2>/dev/null || true
  if sshd -t; then
    systemctl reload ssh
    info "ok: sshd 已重载（仅公钥）"
  else
    warn "sshd 配置有语法错误，未重载 —— 请检查 00-rak-hardening.conf"
  fi
fi

# ------------------------------------------------- 2. 功耗档位 + 时钟钉死
say "2. 功耗档位与时钟（10W 满档 + jetson_clocks 锁频）"
# JetPack 自带的 nvpmodel.service 在"已经是 10W"的镜像上会要求交互确认重启，
# 读不到输入就以 status=234 失败（开机一条红字）。换成非交互脚本：
# 只有在档位真的不等于目标时才切换。
if [ "$MODE" = apply ]; then
  cat > /usr/local/sbin/rak-nvpmodel.sh <<'EOF'
#!/usr/bin/env bash
# 非交互设置 NV 功耗档位。已在目标档位时什么都不做（避免 JetPack 自带脚本
# 因为"需要重启确认"而失败）。
set -u
TARGET="${RAK_NVPMODE:-0}"      # 0 = 10W（Orin Nano DevKit 的最高档）
mode="$(nvpmodel -q 2>/dev/null | tail -1)"
name="$(nvpmodel -q 2>/dev/null | head -1)"
if [ "$mode" = "$TARGET" ]; then
  echo "rak-nvpmodel: 已在档位 $mode（$name），无需切换"
  exit 0
fi
echo "rak-nvpmodel: $name -> 档位 $TARGET"
nvpmodel -m "$TARGET" || echo "rak-nvpmodel: 切换失败，可能需要重启后生效"
EOF
  chmod 0755 /usr/local/sbin/rak-nvpmodel.sh

  mkdir -p /etc/systemd/system/nvpmodel.service.d
  cat > /etc/systemd/system/nvpmodel.service.d/10-rak.conf <<'EOF'
[Service]
# 用非交互版本替换 JetPack 默认的 /etc/systemd/nvpmodel.sh
ExecStart=
ExecStart=/usr/local/sbin/rak-nvpmodel.sh
EOF

  cat > /etc/systemd/system/rak-clocks.service <<'EOF'
[Unit]
Description=RAK: jetson_clocks 锁 CPU/GPU/EMC 到最高频（性能优先基线）
After=nvpmodel.service nvpower.service
Requires=nvpmodel.service

[Service]
Type=oneshot
# jetson_clocks（无参数）= 锁所有时钟到最高 + 把 governor 设为 performance。
# 代价: 功耗/温度上升，nvfancontrol 负责压风扇。收益: 频率不再随负载抖动，
# 控制环的抖动从"频率域"变成"固定值"，基准数字才可复现。
ExecStart=/usr/bin/jetson_clocks
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF
  systemctl daemon-reload
  systemctl enable rak-clocks.service >/dev/null 2>&1 || true
  systemctl restart rak-clocks.service >/dev/null 2>&1 || true
  info "ok: rak-clocks.service 已启用（jetson_clocks）"
fi
# power-profiles-daemon 会把 governor 改回 schedutil，和上面的锁频打架
step_do "停用 power-profiles-daemon（避免改回 schedutil）" systemctl disable --now power-profiles-daemon.service

# ------------------------------------------------------------------ 3. 无头化
say "3. 无头化（把 CMA 还给相机）"
if [ "$MODE" = check ]; then
  info "gdm active=$(systemctl is-active gdm 2>/dev/null) enabled=$(systemctl is-enabled gdm3 2>/dev/null)"
  awk '/CmaTotal|CmaFree|MemAvailable/{printf "   %s %8d kB\n", $1, $2}' /proc/meminfo
else
  systemctl stop gdm 2>/dev/null || true
  systemctl mask gdm3 gdm 2>/dev/null || true
  pkill -9 -f 'gnome-shell|mutter-x11-frames|gnome-software' 2>/dev/null || true
  sleep 2
  info "ok: gdm 已停止并 mask（恢复: scripts/rak-mode.sh desktop）"
fi

# ------------------------------------------------------------ 4. 服务去冗余
say "4. 服务去冗余"
DISABLE_UNITS=(
  bluetooth.service
  # 机器人走 CAN/以太网，不需要蓝牙
  avahi-daemon.service
  # mDNS 广播：机器人不需要被局域网"发现"
  lpd.service
  # 网络打印
  cups.service
  # 打印
  gnome-remote-desktop.service
  # 与 x11vnc 抢 5900；README 实测 vino/gnome-remote 方案已废
  vino.service
  # 同上（L4T 自带的远程桌面，99.7% 帧缓冲重传）
  upower.service
  # 桌面电源管理
  udisks2.service
  # 桌面设备挂载
  colord.service
  # 桌面色彩管理
  accounts-daemon.service
  # 桌面账户
  switcheroo-control.service
  # 桌面显卡切换
  power-profiles-daemon.service
  # 会把 governor 改回 schedutil
  docker.service
  # 没有任何容器在跑，纯占内存
  docker.socket
  containerd.service
  sssd.service
  # LDAP 客户端，机器人不在域里
  openvpn.service
  # 组网用 netmaker，openvpn 不用
  rsync.service
  # 机器人不需要
  iperf3.service
  # 端口常开，按需手动 systemctl start
  lttng-sessiond.service
  # 追踪守护进程，机器人不需要
  netclient.service
  # NVIDIA 网络客户端
  nv-power-adapter-check.service
  nv-tee-supplicant.service
  # TEE 代理，无人使用
  apport.service
  # 崩溃上报会往 /var/crash 落盘
  haveged.service
  # 内核 6.8 自带 jitter entropy，haveged 反而拖慢启动
  rsyslog.service
  # 日志已由 journald 承担
  cloud-init.service
  # 机器不联网初始化；而且它会把 SSH 密码登录重新打开
  cloud-init-local.service
  cloud-config.service
  cloud-final.service
)
for u in "${DISABLE_UNITS[@]}"; do
  step_do "停用 $u" systemctl disable --now "$u"
done

# ---------------------------------------------------------- 5. 实时与内存锁
say "5. 实时优先级与内存锁（limits）"
if [ "$MODE" = apply ]; then
  cat > /etc/security/limits.d/99-rak-rt.conf <<'EOF'
# 机器人核心的实时约束：
#   rtprio 95      —— ROS 2 节点与控制环可以用 SCHED_FIFO（S1 推理/控制线程）
#   memlock        —— 不限：CUDA pinned host 内存（rak_stereo_driver 的
#                     PinnedBuffer）与大页锁定的分配会超过默认的 64 MB 上限，
#                     超出时 cudaHostAlloc 直接失败
#   nice -20       —— 普通进程也能拿到高 niceness
xrak - rtprio 95
xrak - memlock unlimited
xrak - nice -20
xrak - core unlimited
EOF
  info "ok: /etc/security/limits.d/99-rak-rt.conf（重新登录后生效）"
fi

# ------------------------------------------------------ 6. 日志与包管理
say "6. 日志与包管理（机器人不许半夜自动重启）"
if [ "$MODE" = apply ]; then
  mkdir -p /etc/systemd/journald.conf.d
  cat > /etc/systemd/journald.conf.d/99-rak.conf <<'EOF'
[Journal]
# 磁盘日志封顶：机器人常年运行，journald 不封顶会把 NVMe 写满。
SystemMaxUse=256M
SystemKeepFree=2G
MaxRetentionSec=2week
# ROS 节点日志是成串的，默认的 30s/10000 条限流会静默丢日志（"Suppressed N messages"），
# 排障时最需要的就是这些日志，所以放宽。
RateLimitIntervalSec=30s
RateLimitBurst=20000
EOF
  systemctl restart systemd-journald
  info "ok: journald 封顶 256M / 保留 2 周"

  # 自动安全更新会在机器人运行时升级内核并要求重启 —— 对机器人是不可接受的。
  # 改为"人工排期升级"：需要更新时手动 apt upgrade 并在停机窗口重启。
  if [ -f /etc/apt/apt.conf.d/20auto-upgrades ]; then
    sed -i 's/^APT::Periodic::Unattended-Upgrade "1"/APT::Periodic::Unattended-Upgrade "0"/' \
      /etc/apt/apt.conf.d/20auto-upgrades
    info "ok: unattended-upgrades 已关闭（改为人工排期）"
  fi
fi
step_do "启用 fstrim 定时器（NVMe 健康）" systemctl enable --now fstrim.timer

# ------------------------------------------------------- 7. 机器人运行时
say "7. 机器人运行时（CAN / 雷达 udev / VNC）"
if [ "$MODE" = apply ]; then
  # CAN: 仓库里的 can0-up.service 带 Orin CAN pinmux 写入（AON pinmux 重启必丢，
  # 漏写时 can0 照常 UP 但一帧收不到）。装成 rak-can0.service 并开机自启。
  if [ -f "$REPO/scripts/can0-up.service" ]; then
    cp "$REPO/scripts/can0-up.service" /etc/systemd/system/rak-can0.service
    systemctl daemon-reload
    systemctl enable rak-can0.service >/dev/null 2>&1 || true
    info "ok: rak-can0.service 已安装（1 Mbps, sample-point 0.800）"
  fi
  # 雷达别名: /dev/wheeltec_lidar（否则驱动报 open_port ERROR，像硬件坏了）
  if [ -f "$REPO/src/rak_bringup/udev/99-wheeltec-lidar.rules" ]; then
    cp "$REPO/src/rak_bringup/udev/99-wheeltec-lidar.rules" /etc/udev/rules.d/
    udevadm control --reload-rules
    udevadm trigger
    info "ok: 雷达 udev 别名规则已装（/dev/wheeltec_lidar）"
  fi
  # VNC: README 的远程桌面路线（x11vnc 共享真实 X display，不用 vino）
  if command -v x11vnc >/dev/null 2>&1; then
    install -m 0755 "$REPO/scripts/x11vnc-share" /usr/local/sbin/x11vnc-share
    if [ ! -f /etc/x11vnc.pass ]; then
      x11vnc -storepasswd kali /etc/x11vnc.pass >/dev/null 2>&1 || true
      chmod 0600 /etc/x11vnc.pass 2>/dev/null || true
      info "ok: /etc/x11vnc.pass 已创建（团队默认口令）"
    fi
    if [ -f "$REPO/scripts/x11vnc.service" ]; then
      cp "$REPO/scripts/x11vnc.service" /etc/systemd/system/x11vnc.service
      systemctl daemon-reload
      systemctl enable x11vnc.service >/dev/null 2>&1 || true
      info "ok: x11vnc.service 已启用（headless 下自动空转等待 X）"
    fi
  else
    warn "未装 x11vnc，跳过 VNC 配置"
  fi
  # vino 的 autostart 带 X-GNOME-AutoRestart=true，杀进程它也会复活并抢 5900
  if [ -f /home/xrak/.config/autostart/vino-server.desktop ]; then
    mv /home/xrak/.config/autostart/vino-server.desktop \
       /home/xrak/.config/autostart/vino-server.desktop.disabled
    info "ok: vino autostart 已禁用"
  fi
fi

# --------------------------------------------------- 7.5 pip 运行时依赖
say "7.5 pip 运行时依赖（apt 里没有的那几个）"
# 为什么用 --break-system-packages：ament_python 节点跑在**系统 Python** 上，
# 把 vosk 装进用户 venv 里 ROS 节点根本 import 不到（这是 ROS 2 的既定拓扑，
# 不是偷懒）。CLAUDE.md 禁止的是"项目 venv / 通用 CUDA、PyTorch 轮子"，
# 这两只是 apt 缺失的纯 Python 运行期依赖。
if [ "$MODE" = check ]; then
  for m in vosk urdf_parser_py; do
    if python3 -c "import $m" 2>/dev/null; then info "python3 -c 'import $m' -> ok"; else info "python3 -c 'import $m' -> 缺失"; fi
  done
elif [ "$MODE" = dry ]; then
  for m in vosk urdf_parser_py; do
    python3 -c "import $m" 2>/dev/null || info "[dry-run] 将 pip install $m"
  done
else
  MISSING_PIP=()
  for m in vosk urdf_parser_py; do
    python3 -c "import $m" 2>/dev/null || MISSING_PIP+=("$m")
  done
  if [ ${#MISSING_PIP[@]} -gt 0 ]; then
    info "安装 pip 包: ${MISSING_PIP[*]}"
    python3 -m pip install --quiet --break-system-packages "${MISSING_PIP[@]}" \
      && info "ok: pip 包已安装" || warn "pip 安装失败（离线？）—— rak_speech 的 ASR 需要 vosk"
  else
    info "ok: pip 包齐了"
  fi
  # vosk 模型是数据不是包（几十 MB，不入库）：放在 gitignored 的 data/models/
  if command -v python3 >/dev/null && python3 -c "import vosk" 2>/dev/null; then
    if ! ls /home/xrak/rak/data/models/vosk-model-* >/dev/null 2>&1; then
      warn "缺 vosk 模型：把 vosk-model-small-cn-0.22 解压到 data/models/（rak_speech 会自动发现）"
    fi
  fi
fi

# ---------------------------------------------------------------- 8. 汇总
say "8. 基线状态"
awk '/MemAvailable/{printf "  MemAvailable %6d MiB\n", $2/1024}' /proc/meminfo
awk '/CmaTotal/{printf "  CmaTotal     %6d MiB\n", $2/1024}' /proc/meminfo
awk '/CmaFree/{printf "  CmaFree      %6d MiB\n", $2/1024}' /proc/meminfo
info "NV 功耗档位: $(nvpmodel -q 2>/dev/null | head -1)"
info "CPU governor : $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor 2>/dev/null)"
info "SSH          : PasswordAuthentication=$(sshd -T 2>/dev/null | grep -i '^passwordauthentication' | awk '{print $2}')"
info "gdm          : $(systemctl is-active gdm 2>/dev/null) / display-manager=$(systemctl is-enabled gdm3 2>/dev/null)"
echo
echo "基线应用完成。CMA 若仍低，说明还有进程占着显存池（用 dmesg | grep -i cma 查）。"
