#!/usr/bin/env bash
# 回滚 IMU overlay：从 extlinux.conf 移除 FDT/OVERLAYS，删除 dtbo
# 用法: sudo bash ~/jetson-setup/imu/revert.sh
set -euo pipefail

EXTLINUX=/boot/extlinux/extlinux.conf
DTBO=/boot/lsm6dsr-spi.dtbo

cp -a "$EXTLINUX" "$EXTLINUX.bak.revert.$(date +%F-%H%M%S)"
python3 - "$EXTLINUX" <<'PY'
import sys, re
path = sys.argv[1]
txt = open(path).read()
m = re.search(r'(LABEL primary\n(?:[ \t]+.*\n)*)', txt)
if m:
    block = '\n'.join(l for l in m.group(1).splitlines()
                      if not l.strip().startswith(('FDT ', 'OVERLAYS '))) + '\n'
    txt = txt[:m.start(1)] + block + txt[m.end(1):]
    open(path, 'w').write(txt)
    print("✓ 已从 extlinux.conf 移除 FDT/OVERLAYS")
PY
rm -f "$DTBO"
echo "✓ 已删除 $DTBO"
echo "   reboot 后即恢复原状（40-pin SPI 回到 spidev）"
