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
            # 总线级参数（ADR-0024）：帧级参数只在 EDS 里。master.dcf 是
            # 官方 dcfgen 从 bus.yaml + 两份 EDS 生成的派生件，漂移门禁见
            # test/test_dcf_sync.py。
            "config/bus.yaml", "config/master.dcf",
        ]),
    ],
    install_requires=["setuptools", "PyYAML"],
    # colcon test 用 pytest 收集 test/ 下的用例（test_contract.py / test_eds_gate.py）。
    # 没有这一行 setuptools 不认 test 目录，colcon test 以 "NO TESTS RAN" + exit 5
    # 结束 —— 看起来像测试失败，实际是**测试从来没跑过**。
    # rak_intent / rak_lifecycle 都这么配，本包此前漏了。
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="CANopen v5 contract layer (ADR-0022).",
    license="Apache-2.0",
    entry_points={},
)
