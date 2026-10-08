from setuptools import setup

package_name = "rak_drive_tui"

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
    description="Mecanum driving TUI over USB v2 DRIVE_COMMAND with DRIVE_STATE panel.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_tui = rak_drive_tui.drive_tui:main",
        ],
    },
)
