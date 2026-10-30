"""ament_python packaging for rak_metrics."""

from glob import glob
from setuptools import setup

PACKAGE_NAME = "rak_metrics"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=[PACKAGE_NAME],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}/launch", glob("launch/*.launch.py")),
        (f"share/{PACKAGE_NAME}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    extras_require={"test": ["pytest"]},
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rak_metrics = rak_metrics.node:main",
        ],
    },
)
