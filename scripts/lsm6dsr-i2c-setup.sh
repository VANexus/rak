#!/bin/bash
# [AI] LSM6DSRTR i2c 侧准备（由 lsm6dsr-i2c.service 以 root 调用；也可手动 sudo 跑）。
#
# 2026-10-10 两个修复:
#   1) 地址自动探测——SA0 线态漂移会让芯片在 0x6a/0x6b 之间换址（当日实翻 0x6b，
#      服务与节点全部失联）。先探测可应答地址，之后所有操作都用该地址。
#   2) **不创建/解绑内核 i2c 设备**——内核 st_lsm6dsx 驱动一旦绑定，会把自己的
#      accel ODR（CTRL1_XL=0x10，12.5Hz）持续强写回，覆盖我们的 208Hz 配置
#      （实测：绑定期间写 0x50，0.3s 内被回退成 0x10；解绑后 0x50/0x30 均稳定保持）。
#      本项目数据路径是直接 /dev/i2c-7 块读（scripts/iio_imu.py），不需要 IIO 设备，
#      也就没有驱动 runtime-PM 保活问题。
#
# 坑（勿踩）:
#   - CTRL3_C 必须 0x44(BDU+IF_INC)；写 0x40 会清 IF_INC，块读 12 字节全是同一寄存器。
#   - 内核驱动 st_lsm6dsx 的 sysfs 写路径 desync（写 ODR 报成功但寄存器不生效）。
#   - WHO_AM_I(0x0F) 恒回 0x6b，与 I2C 地址无关。
set -u

BUS=7
AD=0x6a
if ! i2cget -f -y $BUS 0x6a 0x0f >/dev/null 2>&1; then
  AD=0x6b
fi

# 清除历史遗留的内核 i2c 设备（旧服务/boot 时创建的；仍绑定中的会被内核先解绑），
# 防止 st_lsm6dsx 驱动绑定后回写 accel ODR
for a in 0x6a 0x6b; do
  echo "$a" > /sys/bus/i2c/devices/i2c-$BUS/delete_device 2>/dev/null || true
done

i2cset -f -y $BUS $AD 0x12 0x44   # CTRL3_C = BDU | IF_INC
i2cset -f -y $BUS $AD 0x10 0x50   # CTRL1_XL = 208 Hz, ±2 g
i2cset -f -y $BUS $AD 0x11 0x50   # CTRL2_G  = 208 Hz, ±250 dps

XL=$(i2cget -f -y $BUS $AD 0x10)
G=$(i2cget -f -y $BUS $AD 0x11)
echo "lsm6dsr ready @ i2c-$BUS/$AD (CTRL3_C=0x44 CTRL1_XL=$XL CTRL2_G=$G)"