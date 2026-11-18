from setuptools import setup


package_name = "rak_road_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@xrak.local",
    description="LFD-RoadSeg inference and optional semantic point-cloud input for Nav2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "roadseg_node = rak_road_perception.roadseg_node:main",
            "road_follow_node = rak_road_perception.road_follow_node:main",
            "road_goal_node = rak_road_perception.road_goal_node:main",
        ],
    },
)
