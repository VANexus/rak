#!/usr/bin/env bash
# reboot 后验证 IMU 是否被内核驱动接管
set -uo pipefail
ok()  { echo -e "  \033[32m✓\033[0m $*"; }
bad() { echo -e "  \033[31m✗\033[0m $*"; }

echo "== LSM6DSRTR 验证 =="
echo "[驱动绑定]"
if [ -e /sys/bus/spi/devices/spi0.0/driver ]; then
  d=$(basename "$(readlink -f /sys/bus/spi/devices/spi0.0/driver)")
  [ "$d" = "st_lsm6dsx_spi" ] && ok "spi0.0 -> $d" || bad "spi0.0 绑定到 $d（期望 st_lsm6dsx_spi）"
else
  bad "spi0.0 未绑定任何驱动"
fi

echo "[IIO 设备]"
ls /sys/bus/iio/devices/ 2>/dev/null | sed 's/^/  /'
for d in /sys/bus/iio/devices/iio:device*; do
  [ -e "$d/name" ] || continue
  n=$(cat "$d/name")
  case "$n" in
    lsm6dsr*|lsm6dso*|lsm6ds*)
      ok "$(basename $d): $n"
      echo "    采样率档位: $(cat $d/sampling_frequency_available 2>/dev/null | head -1)"
      echo "    当前 accel: $(cat $d/in_accel_sampling_frequency 2>/dev/null) Hz"
      echo "    当前 gyro : $(cat $d/in_anglvel_sampling_frequency 2>/dev/null) Hz"
      echo "    中断: $(cat $d/trigger/current_trigger 2>/dev/null || echo '(未绑定 trigger)')"
      ;;
  esac
done

echo "[中断计数]"
grep -i lsm6ds /proc/interrupts 2>/dev/null | sed 's/^/  /' || echo "  (无 —— 检查 pin15 接线)"
echo "[软链]"; ls -l /dev/robot_spi 2>/dev/null | sed 's/^/  /'

echo
echo "提示：确认采样率/量程/触发模式可写"
echo "  cat /sys/bus/iio/devices/iio:device*/sampling_frequency_available"
echo "  echo 6660 > /sys/bus/iio/devices/iio:device0/in_accel_sampling_frequency"
