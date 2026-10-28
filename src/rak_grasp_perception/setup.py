from glob import glob
from setuptools import setup

setup(
    name="rak_grasp_perception", version="0.1.0", packages=["rak_grasp_perception"],
    data_files=[("share/ament_index/resource_index/packages", ["resource/rak_grasp_perception"]),
                ("share/rak_grasp_perception", ["package.xml"]),
                ("share/rak_grasp_perception/launch", glob("launch/*.py")),
                ("share/rak_grasp_perception/config", glob("config/*"))],
    install_requires=["setuptools"], zip_safe=True, maintainer="xrak",
    maintainer_email="xrak@xrak.local", description="Offline point-cloud grasp inspection.",
    # [AI] tests_require 在现代 setuptools 失效；用 extras_require["test"] 让 colcon 走 pytest。
    license="Apache-2.0", extras_require={"test": ["pytest"]},
    entry_points={"console_scripts": [
        "capture_snapshot = rak_grasp_perception.capture:main",
        "infer_graspnet = rak_grasp_perception.infer:main",
        "replay_grasps = rak_grasp_perception.viewer:main",
        "plan_grasp_sim = rak_grasp_perception.planner:main",
    ]},
)
