from setuptools import setup

package_name = "rak_mcp"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bench_mcp.launch.py"]),
        ("share/" + package_name + "/bench", ["bench/mock_chassis.urdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xrak",
    maintainer_email="xrak@todo.todo",
    description="Standard MCP (stdio) tool surface over the robot tool registry.",
    license="Apache-2.0",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "rak_mcp_server = rak_mcp.server:main",
        ],
    },
)
