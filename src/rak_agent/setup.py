"""ament_python packaging for the ROS 2 decision node."""

from setuptools import setup

PACKAGE_NAME = "rak_agent"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    python_requires=">=3.10,<3.13",
    packages=["rak_agent"],
    install_requires=["setuptools"],
    zip_safe=True,
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    entry_points={
        "console_scripts": [
            "agent_node = rak_agent.node:main",
        ],
    },
)
