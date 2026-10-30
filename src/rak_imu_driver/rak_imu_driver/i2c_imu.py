#!/usr/bin/env python3
# [AI] IMU I2C 路径节点 —— LSM6DSRTR @ 40-pin I2C (SDA=3, SCL=5) → sensor_msgs/Imu @1000Hz
#
# 为什么有这条路径:
#   SPI 路径（spi_imu.py）在 2026-10-26 的现场排查中卡在 CS 上：芯片上电时 CS 为高
#   就会选 I2C 模式，而 Jetson 的 SPI CS 空闲时为高。I2C 路径已实测可用
#   （i2cdetect 0x6B、WHO_AM_I=0x6B），且不依赖 CS。
#
# 接线（Jetson 40 针，两列编号 23/24、25/26）:
#   1  (3.3V) ──► VCC
#   6  (GND)  ──► GND
#   3  (SDA)  ──► SDA/SDI
#   5  (SCL)  ──► SCL/SPC
#   17 (3.3V) ──► CS      ← 必须为高才选 I2C 模式（可悬空，芯片内部上拉）
#              SA0/SDO  ──► 悬空 = 地址 0x6B；接 GND = 0x6A
#
# 总线: /dev/i2c-7 = i2c@c250000，实测 400 kHz (Fast Mode)
#
# 安装方向重映射（2026-10-26 实测模块只能竖着装，芯片 +Y 朝上）:
#   axis_map 参数 = 3 个逗号分隔的轴，描述「发布消息的 x/y/z 各取自芯片的哪个轴」。
#   例: -p axis_map:='x,-z,y'  → msg.x=chip.x, msg.y=-chip.z, msg.z=chip.y
#   只影响向量方向，不改量程/比例。
#
#   ★ 默认 '-z,-x,y' 是 **本机实测校准值**（芯片 +Y 朝上、车头 = 芯片 -Z）:
#       X(前) ← -chip.Z   实测: 车头下压 → x = +5.0
#       Y(左) ← -chip.X   实测: 左侧倾斜 → y = +6.0
#       Z(上) ← +chip.Y   实测: 静止     → z = +9.7
#   模块重新安装/换模块后必须重测三轴；想恢复不映射用 -p axis_map:=x,y,z。
#
# 用法: ros2 run rak_imu_driver i2c_imu                 # 起节点（1000Hz）
#       ros2 run rak_imu_driver i2c_imu -- --probe      # 只读探测
#       python3 -m rak_imu_driver.i2c_imu --probe       # 不 source ROS 也能自检
import ctypes
import fcntl
import os
import struct
import sys
import time

try:
    import rclpy
    from diagnostic_msgs.msg import DiagnosticStatus
    from rclpy.lifecycle import TransitionCallbackReturn
    from rclpy.time import Time
    from rak_lifecycle.managed_node import ManagedNode
    from sensor_msgs.msg import Imu
except ImportError:                                    # --probe 模式允许无 ROS 环境
    rclpy = None
    Imu = None
    ManagedNode = object
    TransitionCallbackReturn = None
    DiagnosticStatus = None
    Time = None

# ---- i2c-dev ioctl 常量 (linux/i2c-dev.h) ----
I2C_SLAVE = 0x0703
I2C_RDWR = 0x0707
I2C_M_RD = 0x0001

# ---- LSM6DSR 寄存器（与 spi_imu.py 保持一致）----
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


def parse_axis_map(spec):
    """'x,-z,y' -> [('x',1), ('z',-1), ('y',1)]；非法输入抛 ValueError。"""
    toks = [t.strip() for t in str(spec).split(',')]
    if len(toks) != 3:
        raise ValueError(f"axis_map 需要 3 个轴，收到: {spec!r}")
    out = []
    for t in toks:
        sign = -1 if t.startswith('-') else 1
        axis = t.lstrip('+-').lower()
        if axis not in ('x', 'y', 'z'):
            raise ValueError(f"axis_map 轴名必须是 x/y/z，收到: {t!r}")
        out.append((axis, sign))
    return out


def map_vec(v, amap):
    """v=(x,y,z) 按 amap 重排/取反。"""
    return tuple(sign * v['xyz'.index(axis)] for axis, sign in amap)


class I2cMsg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16),
                ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16),
                ("buf", ctypes.POINTER(ctypes.c_uint8))]


class I2cRdwrIoctlData(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(I2cMsg)),
                ("nmsgs", ctypes.c_uint32)]


class RawI2c:
    """i2c-dev 封装：写寄存器地址 + 重复起始 + 读数据（LSM6DSx 要求）。"""

    def __init__(self, device='/dev/i2c-7', address=0x6B):
        self.address = address
        self.fd = os.open(device, os.O_RDWR)
        fcntl.ioctl(self.fd, I2C_SLAVE, address)

    def read_regs(self, reg, n):
        tx = (ctypes.c_uint8 * 1)(reg)
        rx = (ctypes.c_uint8 * n)()
        msgs = (I2cMsg * 2)()
        msgs[0].addr = self.address
        msgs[0].flags = 0
        msgs[0].len = 1
        msgs[0].buf = ctypes.cast(tx, ctypes.POINTER(ctypes.c_uint8))
        msgs[1].addr = self.address
        msgs[1].flags = I2C_M_RD
        msgs[1].len = n
        msgs[1].buf = ctypes.cast(rx, ctypes.POINTER(ctypes.c_uint8))
        data = I2cRdwrIoctlData(msgs, 2)
        fcntl.ioctl(self.fd, I2C_RDWR, data)
        return bytes(rx)

    def read_reg(self, reg):
        return self.read_regs(reg, 1)[0]

    def write_reg(self, reg, val):
        tx = (ctypes.c_uint8 * 2)(reg & 0x7F, val & 0xFF)
        msg = I2cMsg(self.address, 0, 2,
                     ctypes.cast(tx, ctypes.POINTER(ctypes.c_uint8)))
        data = I2cRdwrIoctlData(ctypes.pointer(msg), 1)
        fcntl.ioctl(self.fd, I2C_RDWR, data)

    def close(self):
        os.close(self.fd)


def probe(i2c):
    """只读探测: WHO_AM_I + 突发/单读一致性。返回 0/1。"""
    who = None
    for _ in range(5):
        try:
            who = i2c.read_reg(REG_WHO_AM_I)
        except OSError as e:
            print(f'I2C 传输失败: {e}')
            return 1
        if who == 0x6B:
            break
        time.sleep(0.05)
    print(f'WHO_AM_I = 0x{who:02x}' + ('  ✓' if who == 0x6B else '  ✗ (期望 0x6b)'))
    if who != 0x6B:
        print('检查: SDA=3 / SCL=5 / VCC=1 / GND=6，且 CS 必须为高（悬空或接 3V3）')
        return 1
    burst = i2c.read_regs(REG_OUT_START, 12)
    single = [i2c.read_reg(REG_OUT_START), i2c.read_reg(REG_OUT_START + 1)]
    ok = list(burst[:2]) == single
    print(f'突发读  12B = {burst.hex(" ")}')
    print(f'单读 0x22/0x23 = {single[0]:02x} {single[1]:02x}  -> '
          f'自增{"一致" if ok else "不一致"}')
    return 0 if ok else 1


class I2cImuNode(ManagedNode):
    """LSM6DSRTR I2C 节点（ADR-0020 managed + ADR-0023 采样时刻打戳）。

    资源在 `on_configure` 打开（设备与寄存器配置），I/O 在 `on_activate` 才开始
    （timer）。`deactivate → activate` 不重开设备，`cleanup` 才 close —— 这是
    ADR-0020 §5 的契约：4 GB 单板上重配硬件也是有代价的。

    **时间戳**（ADR-0023）：`header.stamp` 是**采样时刻**，不是发布时刻。
    寄存器在 ODR 节拍锁存，本轮读到的是"最近一次锁存"的值，因此采样时刻落在
    `[t0 - 1/ODR, t0]` 内（t0 = 发起读的时刻）。取该区间的中点作为估计，
    最大偏差 1/ODR（1660 Hz 时 0.6 ms），远小于旧的"发布时刻"打戳误差
    （timer 抖动 + 调度延迟，实测可达数毫秒且随负载漂移）。
    `stats` 会把 `now - stamp` 作为 `sample_age_ms` 报出来，用于验证这个估计。
    """

    def __init__(self):
        super().__init__('i2c_imu')
        self.i2c = None
        self.pub = None
        self.axis_map = None
        self.frame = 'imu_link'
        self.topic = '/imu/data_raw'
        self.rate = 1000.0
        self.odr = 1660
        self.n = 0
        self.err = 0
        self.t0 = time.monotonic()
        self.last = None
        self.max_gap = 0.0
        self.max_sample_age_ms = 0.0

    # ---------------------------------------------------------------- configure
    def on_configure(self, state):
        super().on_configure(state)
        device = str(self.declare('device', '/dev/i2c-7'))
        address = int(self.declare('address', 0x6B))
        self.rate = float(self.declare('rate_hz', 1000.0))
        self.odr = int(self.declare('odr', 1660))
        self.frame = str(self.declare('frame_id', 'imu_link'))
        self.topic = str(self.declare('topic', '/imu/data_raw'))
        # [AI] 2026-10-26: 模块只能竖装（芯片 +Y 朝上），用轴映射把发布向量转回
        # REP-103（x 前 / y 左 / z 上）。默认值是 **本机实测校准值**，不是通用值。
        try:
            self.axis_map = parse_axis_map(self.declare('axis_map', '-z,-x,y'))
        except ValueError as exc:
            return self.fail(f'axis_map 非法: {exc}')

        self.declare('startup_timeout_s', 5.0)

        if self.odr not in ODR_BITS:
            self.get_logger().warning(f'odr={self.odr} 不在档位表, 回落 1660')
            self.odr = 1660

        try:
            self.i2c = RawI2c(device, address)
            self.get_logger().info(f'I2C 就绪: {device} addr=0x{address:02x}')
            who = None
            deadline = time.monotonic() + float(
                self.get_parameter('startup_timeout_s').value)
            while time.monotonic() < deadline:
                who = self.i2c.read_reg(REG_WHO_AM_I)
                if who == 0x6B:
                    break
                time.sleep(0.1)
            if who != 0x6B:
                raise RuntimeError(
                    f'WHO_AM_I=0x{who:02x} (期望 0x6b): 检查 SDA=3/SCL=5/VCC=1/GND=6，'
                    'CS 必须为高（悬空或接 3V3）')
            self.i2c.write_reg(REG_CTRL3_C, 0x44)          # BDU | IF_INC
            self.i2c.write_reg(REG_CTRL1_XL, ODR_BITS[self.odr])
            self.i2c.write_reg(REG_CTRL2_G, ODR_BITS[self.odr])
            rb = (self.i2c.read_reg(REG_CTRL3_C), self.i2c.read_reg(REG_CTRL1_XL),
                  self.i2c.read_reg(REG_CTRL2_G))
            if rb[1] != ODR_BITS[self.odr] or rb[2] != ODR_BITS[self.odr]:
                raise RuntimeError(f'寄存器回读不符: CTRL3_C=0x{rb[0]:02x} '
                                   f'CTRL1_XL=0x{rb[1]:02x} CTRL2_G=0x{rb[2]:02x}')
        except OSError as exc:
            self._close()
            return self.fail(f'I2C 传输失败: {exc}')
        except RuntimeError as exc:
            self._close()
            return self.fail(str(exc))

        # 未激活不发布：LifecyclePublisher 在 on_activate 之前丢弃 publish()
        self.pub = self.lifecycle_publisher(Imu, self.topic, 10)
        self.report('device', DiagnosticStatus.OK,
                    f'{device} 0x{address:02x} WHO=0x{who:02x} ODR={self.odr}Hz',
                    {'device': device, 'odr_hz': self.odr, 'who_am_i': f'0x{who:02x}'})
        self.get_logger().info(
            f'IMU(I2C) 已配置: WHO=0x{who:02x} ODR={self.odr}Hz rate={self.rate}Hz '
            f'CTRL3_C=0x{rb[0]:02x} CTRL1_XL=0x{rb[1]:02x} CTRL2_G=0x{rb[2]:02x} '
            f'axis_map={self.get_parameter("axis_map").value}')
        return TransitionCallbackReturn.SUCCESS

    # ----------------------------------------------------------------- activate
    def on_activate(self, state):
        ret = super().on_activate(state)
        if ret != TransitionCallbackReturn.SUCCESS:
            return ret
        self.n = 0
        self.err = 0
        self.t0 = time.monotonic()
        self.last = None
        self.max_gap = 0.0
        self.add_timer(1.0 / self.rate, self.step)
        self.add_timer(5.0, self.stats)
        return TransitionCallbackReturn.SUCCESS

    # -------------------------------------------------------------------- work
    def step(self):
        if self.i2c is None:
            return
        # t0 = 发起读的时刻；采样锁存在此之前 0..1/ODR 内（见类 docstring）
        t0 = self.get_clock().now()
        try:
            buf = self.i2c.read_regs(REG_OUT_START, 12)
        except OSError:
            self.err += 1
            if self.err % 100 == 1:
                self.report('i2c', DiagnosticStatus.WARN,
                            f'I2C 读失败累计 {self.err} 次', {'errors': self.err})
            return

        stamp_ns = t0.nanoseconds - int(0.5e9 / self.odr)
        now_mono = time.monotonic()
        if self.last is not None:
            gap = now_mono - self.last
            if gap > self.max_gap:
                self.max_gap = gap
        self.last = now_mono
        age_ms = (self.get_clock().now().nanoseconds - stamp_ns) / 1e6
        if age_ms > self.max_sample_age_ms:
            self.max_sample_age_ms = age_ms

        gx, gy, gz, ax, ay, az = struct.unpack_from('<6h', buf)
        gyro = map_vec((gx * GYRO_SCALE, gy * GYRO_SCALE, gz * GYRO_SCALE), self.axis_map)
        accel = map_vec((ax * ACCEL_SCALE, ay * ACCEL_SCALE, az * ACCEL_SCALE), self.axis_map)
        m = Imu()
        m.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
        m.header.frame_id = self.frame
        m.angular_velocity.x = gyro[0]
        m.angular_velocity.y = gyro[1]
        m.angular_velocity.z = gyro[2]
        m.linear_acceleration.x = accel[0]
        m.linear_acceleration.y = accel[1]
        m.linear_acceleration.z = accel[2]
        m.orientation_covariance[0] = -1.0
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = 0.01
            m.linear_acceleration_covariance[i] = 0.1
        self.publish(m, publisher=self.pub)
        self.mark_seen(self.topic)
        self.n += 1

    def stats(self):
        dt = time.monotonic() - self.t0
        if dt <= 0 or self.n == 0:
            return
        rate = self.n / dt
        values = {
            'rate_hz': f'{rate:.1f}',
            'target_hz': f'{self.rate:.0f}',
            'max_gap_ms': f'{self.max_gap * 1e3:.2f}',
            'max_sample_age_ms': f'{self.max_sample_age_ms:.2f}',
            'i2c_errors': self.err,
            'stamp_source': 'sample',
        }
        level = DiagnosticStatus.OK
        if abs(rate - self.rate) > 0.1 * self.rate or self.err:
            level = DiagnosticStatus.WARN
        self.report('stream', level, f'{rate:.1f}Hz', values)
        self.get_logger().info(
            f'实测 {rate:.1f}Hz (目标 {self.rate:.0f}Hz), '
            f'max_gap={self.max_gap * 1e3:.2f}ms, '
            f'max_sample_age={self.max_sample_age_ms:.2f}ms, i2c_err={self.err}')
        self.n = 0
        self.err = 0
        self.t0 = time.monotonic()
        self.max_gap = 0.0
        self.max_sample_age_ms = 0.0

    # ------------------------------------------------------------------ cleanup
    def on_cleanup(self, state):
        self._close()
        return super().on_cleanup(state)

    def on_shutdown(self, state):
        self._close()
        return super().on_shutdown(state)

    def _close(self):
        if self.i2c is not None:
            try:
                self.i2c.close()
            except OSError:
                pass
            self.i2c = None


def main(args=None):
    if '--probe' in sys.argv:
        dev = '/dev/i2c-7'
        addr = 0x6B
        for a in sys.argv:
            if a.startswith('--device='):
                dev = a.split('=', 1)[1]
            if a.startswith('--address='):
                addr = int(a.split('=', 1)[1], 0)
        i2c = RawI2c(dev, addr)
        print(f'device={dev} addr=0x{addr:02x}')
        sys.exit(probe(i2c))
    if rclpy is None:
        print('需要 ROS 环境: source src/rak_bringup/scripts/environment.sh', file=sys.stderr)
        sys.exit(1)
    rclpy.init(args=args)
    node = I2cImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
