"""ament_python packaging for commissioning/teach tools (ADR-0013)."""

from setuptools import setup

PACKAGE_NAME = "rak_commissioning"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    python_requires=">=3.10,<3.13",
    packages=["rak_commissioning"],
    install_requires=["setuptools"],
    zip_safe=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", ["launch/chassis_test.launch.py"]),
    ],
    entry_points={
        "console_scripts": [
            "chassis_key_test = rak_commissioning.chassis_key_test:main",
            "chassis_tui = rak_commissioning.chassis_tui:main",
            "arm_tui = rak_commissioning.arm_tui:main",
        ],
    },
)
