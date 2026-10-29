from setuptools import setup

package_name = "rak_canopen"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", [
            "config/canopen_contract.yaml", "config/chassis.json", "config/arm.json",
        ]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="CANopen v5 contract layer (ADR-0022).",
    license="Apache-2.0",
    entry_points={},
)
