from setuptools import setup

package_name = "rak_serial_link"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial>=3.5"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Pure serial link for chassis MCU; 8-byte CAN v3 frames over UART.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_link_node = rak_serial_link.serial_node:main",
            "serial_link_cli = rak_serial_link.cli:main",
        ],
    },
)