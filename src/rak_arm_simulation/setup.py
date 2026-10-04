from setuptools import setup

package_name = "rak_arm_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@xrak.local",
    description="Software-only joint trajectory and gripper action servers.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["fake_joint_controllers = rak_arm_simulation.controllers:main"]},
)
