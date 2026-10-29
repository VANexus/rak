"""ament_python packaging for the shared text-intent contract (ADR-0013)."""

from setuptools import setup

PACKAGE_NAME = "rak_intent"

setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    python_requires=">=3.10,<3.13",
    packages=["rak_intent"],
    install_requires=["setuptools"],
    zip_safe=True,
    # [AI] 让 colcon 用 pytest 跑测试（tests_require 在现代 setuptools 已失效）。
    extras_require={"test": ["pytest"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
)
