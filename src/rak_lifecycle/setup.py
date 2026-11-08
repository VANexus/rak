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
