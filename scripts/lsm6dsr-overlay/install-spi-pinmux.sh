#!/bin/bash
# [AI] 2026-10-10: 编译并安装 LSM6DSRTR SPI1 pinmux overlay。
#
# 流程: dtc 编译 .dts → fdtoverlay 预合并进 stock dtb → 生成 /boot/dtb/rak_imu_spi.dtb
#       → 备份 extlinux.conf → primary 标签的 FDT 行改指新 dtb → （重启后生效）。
# 幂等; 回滚 = 把 extlinux.conf 换回备份（备份名打印在最后）。
#
# 用法: sudo bash install-spi-pinmux.sh
set -euo pipefail

SRC="$(dirname "$(readlink -f "$0")")/rak-imu-spi-pinmux.dts"
STOCK=/boot/dtb/kernel_tegra234-p3768-0000+p3767-0004-nv.dtb
OUTDTB=/boot/dtb/rak_imu_spi.dtb
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

[ -f "$SRC" ] || { echo "找不到 overlay 源: $SRC" >&2; exit 1; }
[ -f "$STOCK" ] || { echo "找不到 stock dtb: $STOCK" >&2; exit 1; }

echo "== 1) 编译 overlay =="
dtc -@ -I dts -O dtb -o "$TMP/rak-imu-spi-pinmux.dtbo" "$SRC"

echo "== 2) 预合并进 stock dtb =="
fdtoverlay -i "$STOCK" -o "$TMP/rak_imu_spi.dtb" "$TMP/rak-imu-spi-pinmux.dtbo"

echo "== 3) 校验合并结果 =="
# 注意: 不能 `dtc | grep -q` —— grep -q 命中即退, dtc 吃到 SIGPIPE 返回 141,
# 在 pipefail 下整条管道被判失败（误报）。先解包到文件再查。
dtc -I dtb -O dts "$TMP/rak_imu_spi.dtb" -o "$TMP/check.dts" 2>/dev/null
grep -q 'spi1_sck_pz3' "$TMP/check.dts" \
  || { echo "合并结果缺少 spi1_sck_pz3，中止" >&2; exit 1; }
if grep -q 'tegra-spidev' "$TMP/check.dts"; then
  echo "  合并 OK: spi1 四脚 pinmux 已并入, tegra-spidev 保留"
else
  echo "警告: 合并结果里没有 tegra-spidev（spidev 可能不会被自动绑定）" >&2
fi

echo "== 4) 安装 dtbo + dtb =="
cp "$TMP/rak-imu-spi-pinmux.dtbo" /boot/
cp "$TMP/rak_imu_spi.dtb" "$OUTDTB"

echo "== 5) 更新 extlinux (先备份) =="
BAK="/boot/extlinux/extlinux.conf.pre-spi-$(date +%Y%m%d%H%M%S)"
cp /boot/extlinux/extlinux.conf "$BAK"
# 只动 primary 标签的 FDT 行（6 空格缩进, 精确全行匹配; JetsonIO 标签用 Tab, 不受影响）
sed -i 's|^      FDT /boot/dtb/kernel_tegra234-p3768-0000+p3767-0004-nv\.dtb$|      FDT /boot/dtb/rak_imu_spi.dtb|' \
    /boot/extlinux/extlinux.conf

grep -nE 'LABEL|FDT|OVERLAYS' /boot/extlinux/extlinux.conf
echo
echo "完成。备份: $BAK"
echo "回滚: sudo cp $BAK /boot/extlinux/extlinux.conf"
echo "重启后验证:  sudo grep SPI1 /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins"
echo "            （四脚应显示 claimed by 3210000.spi function spi1）"