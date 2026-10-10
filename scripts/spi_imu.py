#!/usr/bin/env python3
# [AI] IMU SPI 快路径节点 —— LSM6DSRTR @ 40-pin SPI1 块读 → sensor_msgs/Imu @1000Hz
# 数据链: LSM6DSRTR ─(SPI1 mode3 @10MHz, 40针: 23=SCLK 19=MOSI 21=MISO 24=CS0)→
#         /dev/spidev0.0 ioctl 单事务 13 字节(1 命令 + 12 数据) → /imu/data_raw
#
# 设计取舍:
#   - 不绑内核 st_lsm6dsx 驱动（绑定后它会把自己的 ODR 强写回寄存器，覆盖本节点配置，
#     见 docs/HANDOFF-AGENT.md §3.1）；寄存器全裸配置；
#   - ODR 默认 1.66kHz（LSM6DSR 档位里没有恰好 1000Hz：…833 / 1660…），1kHz 读取下
#     每拍几乎都是新样本；BDU 保证 12 字节为同一采样；
#   - 与 iio_imu.py(I2C) 互为备份路径；topic/单位/量程一致（±2g / ±250dps）。
#
# 用法: python3 spi_imu.py            # 起节点（1000Hz）
#       python3 spi_imu.py --probe    # 只做只读探测(WHO_AM_I + 突发读一致性)后退出
import ctypes
import fcntl
import os
import struct
import sys
import time

# ROS 依赖延迟可选: --probe 是纯硬件只读自检, 不要求先 source ROS 环境
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
except ImportError:                                    # --probe 模式允许无 ROS 环境
    rclpy = None
    Node = object
    Imu = None

# ---- spidev ioctl 常量 (linux/spi/spidev.h) ----
SPI_IOC_WR_MODE = 0x40016b01
SPI_IOC_RD_MODE = 0x80016b01
SPI_IOC_WR_BITS_PER_WORD = 0x40016b03
SPI_IOC_RD_BITS_PER_WORD = 0x80016b03
SPI_IOC_WR_MAX_SPEED_HZ = 0x40046b04
SPI_IOC_RD_MAX_SPEED_HZ = 0x80046b04
SPI_IOC_MESSAGE_1 = 0x40206b00          # _IOW(0x6B, 0, sizeof(struct spi_ioc_transfer)=32)

# ---- LSM6DSR 寄存器 ----
REG_WHO_AM_I = 0x0F
REG_CTRL1_XL = 0x10
REG_CTRL2_G = 0x11
REG_CTRL3_C = 0x12
REG_OUT_START = 0x22                    # OUTX_L_G .. OUTZ_H_A 12 字节

ODR_BITS = {12.5: 0x10, 26: 0x20, 52: 0x30, 104: 0x40, 208: 0x50,
            416: 0x60, 833: 0x70, 1660: 0x80, 3330: 0x90, 6660: 0xA0}

# ±2g → 0.061 mg/LSB；±250dps → 8.75 mdps/LSB
ACCEL_SCALE = 0.000598205
GYRO_SCALE = 0.000152716


class SpiIocTransfer(ctypes.Structure):
    _fields_ = [("tx_buf", ctypes.c_uint64), ("rx_buf", ctypes.c_uint64),
                ("len", ctypes.c_uint32), ("speed_hz", ctypes.c_uint32),
                ("delay_usecs", ctypes.c_uint16), ("bits_per_word", ctypes.c_uint8),
                ("cs_change", ctypes.c_uint8), ("tx_nbits", ctypes.c_uint8),
                ("rx_nbits", ctypes.c_uint8), ("word_delay_usecs", ctypes.c_uint8),
                ("pad", ctypes.c_uint8)]


class RawSpi:
    """spidev 封装：单事务全双工（CS 全程拉低，满足 LSM6DSx 突发读要求）。"""

    def __init__(self, device, mode=3, speed_hz=10_000_000, bits=8):
        self.fd = os.open(device, os.O_RDWR)
        fcntl.ioctl(self.fd, SPI_IOC_WR_MODE, struct.pack("B", mode))
        fcntl.ioctl(self.fd, SPI_IOC_WR_BITS_PER_WORD, struct.pack("B", bits))
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", speed_hz))
        self.mode = self._get_buf(SPI_IOC_RD_MODE, 1)[0]
        self.bits = self._get_buf(SPI_IOC_RD_BITS_PER_WORD, 1)[0]
        self.speed = struct.unpack("I", self._get_buf(SPI_IOC_RD_MAX_SPEED_HZ, 4))[0]
        self._tx = (ctypes.c_ubyte * 64)()
        self._rx = (ctypes.c_ubyte * 64)()

    def _get_buf(self, req, n):
        buf = ctypes.create_string_buffer(n)
        fcntl.ioctl(self.fd, req, buf)
        return buf.raw

    def set_speed(self, speed_hz):
        fcntl.ioctl(self.fd, SPI_IOC_WR_MAX_SPEED_HZ, struct.pack("I", speed_hz))
        self.speed = struct.unpack("I", self._get_buf(SPI_IOC_RD_MAX_SPEED_HZ, 4))[0]

    def xfer(self, payload):
        n = len(payload)
        for i, b in enumerate(payload):
            self._tx[i] = b
        x = SpiIocTransfer()
        x.tx_buf = ctypes.addressof(self._tx)
        x.rx_buf = ctypes.addressof(self._rx)
        x.len = n
        x.speed_hz = self.speed
        x.bits_per_word = self.bits
        fcntl.ioctl(self.fd, SPI_IOC_MESSAGE_1, x)
        return bytes(self._rx[:n])

    def read_reg(self, reg):
        """单字节读: 0x80=读; 响应在第二字节。"""
        return self.xfer(bytes([reg | 0x80, 0x00]))[1]

    def read_regs(self, reg, n):
        """突发读: 0x80=读 + 0x40=地址自增; 12 字节 OUT 一次事务。"""
        return self.xfer(bytes([reg | 0xC0]) + bytes(n))[1:1 + n]

    def write_reg(self, reg, val):
        self.xfer(bytes([reg & 0x7F, val & 0xFF]))


def probe(spi, speeds=(10_000_000, 4_000_000, 1_000_000, 400_000)):
    """只读探测: 逐档速率试 WHO_AM_I（诊断接线/信号完整性）→ 突发读/单读一致性。返回 0/1。"""
    who = None
    for spd in speeds:
        spi.set_speed(spd)
        for _ in range(3):
            try:
                who = spi.read_reg(REG_WHO_AM_I)
            except OSError as e:
                print(f"SPI 传输失败: {e}")
                return 1
            if who == 0x6B:
                break
            time.sleep(0.05)
        print(f"  {spd / 1e6:g} MHz -> WHO_AM_I = 0x{who:02x}" + ("  ✓" if who == 0x6B else ""))
        if who == 0x6B:
            break
    if who != 0x6B:
        print("所有速率均读不到 0x6b。逐项检查: 23=SCLK / 19=MOSI / 21=MISO / 24=CS,"
              " 共地; 若引脚确定接对, 做 19↔21 短接后跑 --loopback 定位是 Jetson 侧还是模块侧。")
        return 1
    burst = spi.read_regs(REG_OUT_START, 12)
    single = [spi.read_reg(REG_OUT_START), spi.read_reg(REG_OUT_START + 1)]
    ok = list(burst[:2]) == single
    print(f"突发读  12B = {burst.hex(' ')}")
    print(f"单读 0x22/0x23 = {single[0]:02x} {single[1]:02x}  -> 自增{'一致' if ok else '不一致(检查 0x40 自增位/IF_INC?)'}")
    return 0 if ok else 1


def loopback(spi, pattern=(0xA5, 0x5A, 0xC3, 0x3C), speed_hz=1_000_000):
    """环回自测: 需先把 40 针 19(MOSI) 与 21(MISO) 短接。验证 Jetson 侧
    (spidev→控制器→引脚) 全通; 读回 == 发出 → 侧问题在模块/接线, 不在 Jetson。返回 0/1。"""
    spi.set_speed(speed_hz)
    rx = spi.xfer(bytes(pattern))
    ok = rx == bytes(pattern)
    print(f"loopback @{speed_hz / 1e6:g}MHz  tx={bytes(pattern).hex(' ')}")
    print(f"                     rx={rx.hex(' ')}  -> {'环路通 ✓ (Jetson 侧 OK)' if ok else '未通 (检查 19/21 短接与 pinmux)'}")
    return 0 if ok else 1


class SpiImuNode(Node):
    def __init__(self):
        super().__init__('spi_imu')
        self.declare_parameter('device', '/dev/spidev0.0')
        self.declare_parameter('mode', 3)
        self.declare_parameter('speed_hz', 10_000_000)
        self.declare_parameter('rate_hz', 1000.0)
        self.declare_parameter('odr', 1660)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')

        device = str(self.get_parameter('device').value)
        self.spi = RawSpi(device,
                          int(self.get_parameter('mode').value),
                          int(self.get_parameter('speed_hz').value))
        self.get_logger().info(
            f'SPI 就绪: {device} mode={self.spi.mode} speed={self.spi.speed} bits={self.spi.bits}')

        # WHO_AM_I 重试（支持上电稍后于节点启动的场景）
        who = None
        for _ in range(50):
            who = self.spi.read_reg(REG_WHO_AM_I)
            if who == 0x6B:
                break
            time.sleep(0.1)
        if who != 0x6B:
            raise RuntimeError(
                f'WHO_AM_I=0x{who:02x} (期望 0x6b): 检查接线(19/21/23/24)与 pinmux'
                ' (sudo grep SPI1 /sys/kernel/debug/pinctrl/2430000.pinmux/pinmux-pins)')

        # 裸配置: CTRL3_C=BDU|IF_INC; CTRL1_XL/CTRL2_G=ODR + ±2g/±250dps
        odr = int(self.get_parameter('odr').value)
        if odr not in ODR_BITS:
            self.get_logger().warning(f'odr={odr} 不在档位表, 回落 1660')
            odr = 1660
        self.spi.write_reg(REG_CTRL3_C, 0x44)
        self.spi.write_reg(REG_CTRL1_XL, ODR_BITS[odr])
        self.spi.write_reg(REG_CTRL2_G, ODR_BITS[odr])
        rb = (self.spi.read_reg(REG_CTRL3_C), self.spi.read_reg(REG_CTRL1_XL),
              self.spi.read_reg(REG_CTRL2_G))
        if rb[1] != ODR_BITS[odr] or rb[2] != ODR_BITS[odr]:
            raise RuntimeError(f'寄存器回读不符: CTRL3_C=0x{rb[0]:02x} '
                               f'CTRL1_XL=0x{rb[1]:02x} CTRL2_G=0x{rb[2]:02x}')

        rate = float(self.get_parameter('rate_hz').value)
        self.pub = self.create_publisher(Imu, str(self.get_parameter('topic').value), 10)
        self.frame = str(self.get_parameter('frame_id').value)
        self.err = 0
        self.n = 0
        self.t0 = time.monotonic()
        self.last = None
        self.max_gap = 0.0
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.stats_timer = self.create_timer(5.0, self.stats)
        self.get_logger().info(
            f'IMU(SPI) 节点就绪: WHO=0x{who:02x} ODR={odr}Hz rate={rate}Hz '
            f'CTRL3_C=0x{rb[0]:02x} CTRL1_XL=0x{rb[1]:02x} CTRL2_G=0x{rb[2]:02x}')

    def step(self):
        now = time.monotonic()
        if self.last is not None:
            gap = now - self.last
            if gap > self.max_gap:
                self.max_gap = gap
        self.last = now
        try:
            buf = self.spi.read_regs(REG_OUT_START, 12)
        except OSError:
            self.err += 1
            if self.err % 100 == 1:
                self.get_logger().warning(f'SPI 读失败 x{self.err}')
            return
        gx, gy, gz, ax, ay, az = struct.unpack_from('<6h', buf)
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.angular_velocity.x = gx * GYRO_SCALE
        m.angular_velocity.y = gy * GYRO_SCALE
        m.angular_velocity.z = gz * GYRO_SCALE
        m.linear_acceleration.x = ax * ACCEL_SCALE
        m.linear_acceleration.y = ay * ACCEL_SCALE
        m.linear_acceleration.z = az * ACCEL_SCALE
        m.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = 0.01
            m.linear_acceleration_covariance[i] = 0.1
        self.pub.publish(m)
        self.n += 1

    def stats(self):
        dt = time.monotonic() - self.t0
        if dt <= 0 or self.n == 0:
            return
        self.get_logger().info(
            f'实测 {self.n / dt:.1f}Hz (目标 {1.0 / self.timer.timer_period_ns * 1e9:.0f}Hz), '
            f'max_gap={self.max_gap * 1e3:.2f}ms, spi_err={self.err}')
        self.n = 0
        self.t0 = time.monotonic()
        self.max_gap = 0.0


def main(args=None):
    if '--probe' in sys.argv or '--loopback' in sys.argv:
        dev = '/dev/spidev0.0'
        for a in sys.argv:
            if a.startswith('--device='):
                dev = a.split('=', 1)[1]
        spi = RawSpi(dev)
        print(f'device={dev} mode={spi.mode} speed={spi.speed} bits={spi.bits}')
        rc = loopback(spi) if '--loopback' in sys.argv else probe(spi)
        sys.exit(rc)
    if rclpy is None:
        print('需要 ROS 环境: source src/rak_bringup/scripts/environment.sh', file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=args)
    node = SpiImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()