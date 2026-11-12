#!/usr/bin/env bash
# rak-mode.sh -- switch the Jetson between local-display and headless modes.
#
# Why this exists: on a 4 GB Orin Nano the display stack is not a rounding error,
# and it is not just RAM. Measured on this board (2026-10-02):
#
#   logged-in GNOME session : MemAvailable ~1.8 GiB, CmaFree      0 kB
#   gdm stopped             : MemAvailable ~2.3 GiB, CmaFree  75-160 MiB
#
# The CMA number is the one that bites: eye captures through V4L2 with
# V4L2_MEMORY_MMAP and requests 4 buffers (capture_node.cpp), which videobuf2
# serves from that same pool. With the desktop up, VIDIOC_REQBUFS has no room to
# succeed -- the capture path fails for a reason that looks like a camera bug.
#
# headless also *masks* the display manager, so a reboot does not silently take
# the pool back. Mask rather than disable because on this image `gdm.service` is
# a static unit -- `systemctl disable gdm` is a no-op -- and what actually pulls
# it up at boot is the `display-manager.service` -> `gdm3.service` alias symlink.
# The local touchscreen goes dark and every `DISPLAY=:1` launch script
# (run_slam.sh / run_nav2.sh / rviz there) stops working -- view rviz from the
# workstation instead, or run `desktop` when you need the screen.
#
#   sudo scripts/rak-mode.sh status
#   sudo scripts/rak-mode.sh headless
#   sudo scripts/rak-mode.sh desktop
#
# Needs root only for systemctl and drop_caches; nothing here deletes anything.

set -euo pipefail

report() {
  awk '/MemAvailable/{printf "  MemAvailable %6d MiB\n", $2/1024}' /proc/meminfo
  awk '/CmaTotal/{printf "  CmaTotal     %6d MiB\n", $2/1024}' /proc/meminfo
  awk '/CmaFree/{printf "  CmaFree      %6d MiB\n", $2/1024}' /proc/meminfo
  local cf
  cf=$(awk '/CmaFree/{print $2}' /proc/meminfo)
  # eye needs 4 MMAP buffers; at 1280x720 MJPEG the driver rounds each up to at
  # least the raw frame size, so ask for ~16 MiB and call the margin honest.
  if [ "$cf" -lt 16384 ]; then
    # 2026-11-13: CmaFree 低**不等于**相机采不到。CMA 池允许页缓存临时借用，
    # 真来 cma_alloc 时内核会把那些页迁走。区分方法就是掉一次缓存看它回不回来
    # （本机实测：稳态 6 MB -> 掉缓存后 140 MB，且能稳住）。
    # 所以这里先给原始值，再给判据，而不是直接断言"视觉采集会失败"。
    local cf2
    sync 2>/dev/null || true
    if [ "$(id -u)" -eq 0 ]; then
      echo 3 > /proc/sys/vm/drop_caches 2>/dev/null || true
    fi
    cf2=$(awk '/CmaFree/{print $2}' /proc/meminfo)
    if [ "$cf2" -ge 16384 ]; then
      echo "  -> CMA 名义上低（$((cf/1024)) MiB），但掉缓存后回到 $((cf2/1024)) MiB："
      echo "     属于页缓存借用（可迁移），相机申请缓冲时会自动腾出。不是故障。"
    else
      echo "  -> CMA 掉缓存后仍只有 $((cf2/1024)) MiB：有进程持有不可迁移的 DMA 缓冲。"
      echo "     先查是不是桌面开着（scripts/rak-mode.sh desktop），再看"
      echo "     sudo ls /sys/kernel/debug/nvmap/handles_by_pid 与 dmesg | grep -i cma"
    fi
  else
    echo "  -> CMA 余量可用"
  fi
  printf "  gdm: active=%s  enabled=%s\n" "$(systemctl is-active gdm 2>/dev/null || true)" "$(systemctl is-enabled gdm 2>/dev/null || true)"
  # The unit that decides whether a reboot brings the display manager back.
  printf "  display-manager -> gdm3: %s\n" "$(systemctl is-enabled gdm3 2>/dev/null || true)"
  printf "  图形会话: %s\n" "$(loginctl list-sessions --no-pager 2>/dev/null | awk 'NR>1 && $0 !~ /closing/' | wc -l)"
}

case "${1:-status}" in
  status)
    report
    ;;
  headless)
    echo "== 停止并 mask 显示管理器（触摸屏会黑，DISPLAY=:1 的脚本失效）=="
    systemctl stop gdm
    # gdm.service is static here, so masking it alone would not survive a reboot:
    # gdm3 is the unit the boot actually reaches through display-manager.service.
    systemctl mask gdm3 gdm 2>/dev/null || systemctl mask gdm3 2>/dev/null || true
    pkill -9 -f 'gnome-shell|mutter-x11-frames|gnome-software' 2>/dev/null || true
    sleep 3
    sync; echo 3 > /proc/sys/vm/drop_caches
    report
    echo
    echo "恢复: sudo scripts/rak-mode.sh desktop"
    ;;
  desktop)
    echo "== 解除 mask 并启动 gdm（回到登录界面；自动登录未开，需要人在触摸屏输一次密码）=="
    systemctl unmask gdm3 gdm 2>/dev/null || systemctl unmask gdm3 2>/dev/null || true
    systemctl start gdm
    sleep 5
    report
    ;;
  *)
    echo "用法: $0 status|headless|desktop" >&2
    exit 2
    ;;
esac
