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
            # EDS 副本必须一起装：门禁比较的就是"契约派生 vs 同一份 EDS"，
            # 只装 JSON 的话安装态跑 eds_gate 必然报"缺少 EDS 副本"。
            "config/chassis.eds", "config/arm.eds",
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
