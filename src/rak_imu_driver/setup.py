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
    # [AI] 让 colcon 用 pytest 跑测试（tests_require 在现代 setuptools 已失效）。
    extras_require={"test": ["pytest"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            # [AI] 正式化后的 IMU 节点入口（原 scripts/spi_imu.py 裸脚本）。
            # --probe / --loopback 只读自检仍可用:  ros2 run rak_imu_driver spi_imu -- --probe
            "spi_imu = rak_imu_driver.spi_imu:main",
        ],
    },
)
