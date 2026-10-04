from setuptools import setup

package_name = "rak_task_executor"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/config",
            ["config/waypoints.yaml", "config/waypoints_sim.yaml"],
        ),
        (
            "share/" + package_name + "/launch",
            ["launch/task_simulation.launch.py"],
        ),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Single owner for text navigation tasks and Nav2 interruption lifecycle.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "task_executor = rak_task_executor.node:main",
            "task_cli = rak_task_executor.console:main",
            "fake_navigation = rak_task_executor.sim_nav:main",
        ],
    },
)
