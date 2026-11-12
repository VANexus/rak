from setuptools import setup

package_name = "rak_lifecycle"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    # colcon test 用 pytest 收集 test/ 下的用例。没有这一行，setuptools 不认
    # test 目录，colcon test 会以 "NO TESTS RAN" + exit code 5 结束 —— 看起来像
    # 测试失败，其实是测试**从来没跑过**。rak_intent 早就这么配了，本包漏了。
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description=(
        "Managed-lifecycle infrastructure (ADR-0020): fault-domain manifest, ManagedNode base, "
        "system manager and operator CLI. Foundation package — depends on no other rak package."
    ),
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "system_manager = rak_lifecycle.system_manager:main",
            "rak_system = rak_lifecycle.cli:main",
        ],
    },
)
