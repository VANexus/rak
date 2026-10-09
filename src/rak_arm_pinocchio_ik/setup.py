"""rak_arm_pinocchio_ik — Pinocchio-backed IK service for MoveIt.

Plugin chain (no C++ pluginlib, no colcon rebuild loop):

    MoveIt move_group
      -> SrvKinematicsPlugin (apt: ros-jazzy-moveit-kinematics, .so ships in /opt/ros/jazzy/lib)
        -> this service (moveit_msgs/srv/GetPositionIK, GetKinematicSolverInfo)
          -> pinocchio Python (apt: ros-jazzy-pinocchio, on sys.path after sourcing Jazzy)

Why a service plugin and not a Python pluginlib KinematicsBase subclass:
MoveIt 2 Jazzy does not expose `moveit.core.kinematics_base.KinematicsBase`
as an importable Python class (`from moveit.core import kinematics_base`
fails). SrvKinematicsPlugin avoids that entirely by delegating IK to a
plain ROS service; the math is independent of MoveIt's plugin machinery.
"""

from setuptools import find_packages, setup

package_name = "rak_arm_pinocchio_ik"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pinocchio_ik.launch.py"]),
        ("share/" + package_name + "/config", ["config/rak_arm_pinocchio_ik.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@xrak.local",
    description="Pinocchio-backed IK service for MoveIt (srv_kinematics_plugin glue).",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_service_node = rak_arm_pinocchio_ik.ik_service_node:main",
        ],
    },
)
