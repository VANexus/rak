"""ament_python packaging for the Jetson-direct SPI IMU driver (ADR-0002)."""

from setuptools import setup

PACKAGE_NAME = "rak_imu_driver"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    python_requires=">=3.10,<3.13",
    packages=[PACKAGE_NAME],
    install_requires=["setuptools"],
    zip_safe=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            # [AI] 正式化后的 IMU 节点入口（原 scripts/spi_imu.py 裸脚本）。
            # --probe / --loopback 只读自检仍可用:  ros2 run rak_imu_driver spi_imu -- --probe
            "spi_imu = rak_imu_driver.spi_imu:main",
            # [AI] 2026-10-26: I2C 路径（SPI 的 CS 上电电平问题无法在 Jetson 侧修复）。
            # 接线 SDA=3 / SCL=5 / VCC=1 / GND=6，CS 悬空或接 3V3；地址 0x6B。
            "i2c_imu = rak_imu_driver.i2c_imu:main",
        ],
    },
)
