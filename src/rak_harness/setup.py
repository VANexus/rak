from glob import glob
import os

from setuptools import find_packages, setup

package_name = "rak_harness"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Jev/task-executor arbitration seam and voice dialogue loop",
    license="Apache-2.0",
    # [AI] tests_require 在现代 setuptools 失效；用 extras_require["test"] 让 colcon 走 pytest。
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "harness_node = rak_harness.node:main",
        ]
    },
)
