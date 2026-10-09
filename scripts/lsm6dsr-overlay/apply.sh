#!/usr/bin/env bash
# 安装 LSM6DSRTR 内核驱动 overlay 并挂到 extlinux.conf
# 用法: sudo bash ~/jetson-setup/imu/apply.sh
# 之后需 reboot。回滚: sudo bash ~/jetson-setup/imu/revert.sh
set -euo pipefail

DIR="$(cd "$(dirname "$0")" && pwd)"
EXTLINUX=/boot/extlinux/extlinux.conf
DTB=/boot/dtb/kernel_tegra234-p3768-0000+p3767-0004-nv.dtb
DTBO=/boot/lsm6dsr-spi.dtbo

[ -f "$DTB" ] || { echo "✗ 找不到 base DTB: $DTB"; exit 1; }
[ -f "$EXTLINUX" ] || { echo "✗ 找不到 $EXTLINUX"; exit 1; }

echo "[1/4] 编译 overlay"
dtc -@ -I dts -O dtb -o /tmp/lsm6dsr-spi.dtbo "$DIR/lsm6dsr-spi.dts"

echo "[2/4] 非破坏性验证（合并进 base DTB 副本）"
fdtoverlay -i "$DTB" -o /tmp/merged-verify.dtb /tmp/lsm6dsr-spi.dtbo
echo "      ✓ 合并成功"

echo "[3/4] 安装 $DTBO"
install -m 644 /tmp/lsm6dsr-spi.dtbo "$DTBO"

echo "[4/4] 更新 $EXTLINUX（备份 .bak.时间戳）"
cp -a "$EXTLINUX" "$EXTLINUX.bak.$(date +%F-%H%M%S)"
python3 - "$EXTLINUX" "$DTB" "$DTBO" <<'PY'
import sys, re
path, dtb, dtbo = sys.argv[1:4]
txt = open(path).read()
m = re.search(r'(LABEL primary\n(?:[ \t]+.*\n)*)', txt)
if not m:
    sys.exit("✗ 未找到 LABEL primary 块")
block = m.group(1)
# 去掉块内已有的 FDT / OVERLAYS，避免重复
block = '\n'.join(l for l in block.splitlines()
                  if not l.strip().startswith(('FDT ', 'OVERLAYS '))) + '\n'
block = re.sub(r'(^[ \t]+LINUX [^\n]*\n)',
               r'\1      FDT %s\n      OVERLAYS %s\n' % (dtb, dtbo),
               block, count=1, flags=re.M)
open(path, 'w').write(txt[:m.start(1)] + block + txt[m.end(1):])
print("      ✓ extlinux.conf 已更新")
PY

echo
echo "=== 新的 primary 块 ==="
awk '/LABEL primary/,/^$/' "$EXTLINUX" | sed 's/^/  /'
echo
echo "✓ 完成。请 reboot 后运行: sudo bash $DIR/verify.sh"
echo "  若起不来，UEFI 菜单选原 primary，或参考 revert.sh"
