from setuptools import setup

package_name = "rak_arm_tui"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Hand-teach TUI for the six Rak arm servos over USB v2 ARM_COMMAND.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_servo_tui = rak_arm_tui.servo_tui:main",
        ],
    },
)
