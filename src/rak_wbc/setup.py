from setuptools import setup

package_name = "rak_wbc"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/wbc_params.yaml"]),
        ("share/" + package_name + "/launch", ["launch/wbc.launch.py"]),
        ("share/" + package_name + "/urdf", ["urdf/wbc_base.urdf.xacro"]),
    ],
    # [AI] 运行期还需要 Pinocchio（PyPI `pin`）——rosdep 键待核实前不写在这里，
    # 见 package.xml 注释与 ADR-0011 待闭合 #6。
    install_requires=["setuptools", "numpy", "osqp"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Velocity-level whole-body arbitration (WBC/HQP); single regular motion writer (ADR-0011).",
    license="Apache-2.0",
    # [AI] tests_require 在现代 setuptools 失效；用 extras_require["test"] 让 colcon 走 pytest。
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "wbc_node = rak_wbc.node:main",
            # [AI] 臂位置流的软件夹具（仅测试/夹具用途，不进生产链路；见文件头注）
            "fake_arm_stream = rak_wbc.fake_arm_stream:main",
        ],
    },
)
