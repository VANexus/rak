#!/usr/bin/env python3
# [AI] IMU 驱动节点(v2, 快路径) —— 一次 I2C 块读拿 6 轴 → sensor_msgs/Imu @100Hz
# 数据链: LSM6DSRTR ─(I2C i2c-7@0x6a)→ /dev/i2c-7 ioctl 块读(绕过驱动 oneshot 等待)
# 内核驱动(st_lsm6dsx_i2c)仍负责: 芯片初始化/量程/ODR(208Hz)/保活(lsm6dsr-i2c.service)
# 本节点只做数据搬运: 0x22-0x2D 12字节 = gyro xyz + accel xyz (int16 LE)
# 单位: 原始值 × sysfs 的 scale(与驱动量程一致) → SI
# 待办(标定): 安装姿态与零偏; 时间戳暂用宿主时钟(内核时间戳需 DTB+INT 硬件触发路径)
import ctypes
import fcntl
import glob
import os
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

I2C_M_RD = 0x0001
I2C_RDWR = 0x0707
OUT_REG = 0x22          # OUTX_L_G..OUTZ_H_A 共 12 字节


class I2cMsg(ctypes.Structure):
    _fields_ = [("addr", ctypes.c_uint16), ("flags", ctypes.c_uint16),
                ("len", ctypes.c_uint16), ("buf", ctypes.c_void_p)]


class I2cRdwrIoctlData(ctypes.Structure):
    _fields_ = [("msgs", ctypes.POINTER(I2cMsg)), ("nmsgs", ctypes.c_uint32)]


class RawI2c:
    """一次事务: 写寄存器指针 + 连读 n 字节(自动增量)"""

    def __init__(self, bus, addr):
        self.fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        self.addr = addr

    def read_regs(self, reg, n):
        w = ctypes.create_string_buffer(bytes([reg]), 1)
        r = ctypes.create_string_buffer(n)
        msgs = (I2cMsg * 2)()
        msgs[0] = I2cMsg(self.addr, 0, 1, ctypes.cast(w, ctypes.c_void_p))
        msgs[1] = I2cMsg(self.addr, I2C_M_RD, n, ctypes.cast(r, ctypes.c_void_p))
        data = I2cRdwrIoctlData(ctypes.cast(msgs, ctypes.POINTER(I2cMsg)), 2)
        fcntl.ioctl(self.fd, I2C_RDWR, data)
        return r.raw


def find_iio(name_sub):
    for d in sorted(glob.glob('/sys/bus/iio/devices/iio:device*')):
        try:
            with open(f'{d}/name') as f:
                if name_sub in f.read():
                    return d
        except OSError:
            pass
    return None


def read_float(path, fallback):
    try:
        with open(path) as f:
            return float(f.read())
    except (OSError, ValueError):
        return fallback


class ImuNode(Node):
    def __init__(self):
        super().__init__('iio_imu')
        self.declare_parameter('bus', 7)
        self.declare_parameter('addr', 0x6a)
        self.declare_parameter('rate_hz', 100.0)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', '/imu/data_raw')
        gyro = find_iio('gyro')
        accel = find_iio('accel')
        base_g, base_a = 0.000152716, 0.000598205     # 驱动默认量程兜底值
        self.g_scale = read_float(f'{gyro}/in_anglvel_scale', base_g) if gyro else base_g
        self.a_scale = read_float(f'{accel}/in_accel_scale', base_a) if accel else base_a
        self.bus = RawI2c(int(self.get_parameter('bus').value),
                          int(self.get_parameter('addr').value))
        self.pub = self.create_publisher(Imu, str(self.get_parameter('topic').value), 10)
        self.frame = str(self.get_parameter('frame_id').value)
        self.fail = 0
        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.step)
        self.get_logger().info(
            f'IMU 节点就绪(快路径): /dev/i2c-{self.get_parameter("bus").value}@0x{int(self.get_parameter("addr").value):02x} '
            f'| gyro_scale={self.g_scale:g} accel_scale={self.a_scale:g} | gyro_dev={gyro}')

    def step(self):
        try:
            buf = self.bus.read_regs(OUT_REG, 12)
        except OSError:
            self.fail += 1
            if self.fail % 100 == 1:
                self.get_logger().warning(f'I2C 读失败 x{self.fail}')
            return
        gx, gy, gz, ax, ay, az = struct.unpack_from('<6h', buf)
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.angular_velocity.x = gx * self.g_scale
        m.angular_velocity.y = gy * self.g_scale
        m.angular_velocity.z = gz * self.g_scale
        m.linear_acceleration.x = ax * self.a_scale
        m.linear_acceleration.y = ay * self.a_scale
        m.linear_acceleration.z = az * self.a_scale
        m.orientation_covariance[0] = -1.0            # 无姿态输出(交 EKF)
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = 0.01
            m.linear_acceleration_covariance[i] = 0.1
        self.pub.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = ImuNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()