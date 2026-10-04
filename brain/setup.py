"""ament_python packaging for the decision layer.

Project metadata (name, version, dependencies, the `laya`/`rai` extras) stays
in pyproject.toml so that `pip install -e ".[laya]"` and `colcon build` read
one source of truth. Only the ament-specific bits live here: the resource
index marker, package.xml, and the console script.

`agent_node` is a stdlib-only bootstrap rather than a console_script entry
point: colcon runs setup.py under its own system python3, so an entry point
would be shebanged to an interpreter that has no pydantic. The bootstrap
re-execs one that does -- see scripts/agent_node.
"""

from setuptools import setup

PACKAGE_NAME = "rak_agent"
PYTHON_REQUIRES = ">=3.10,<3.13"  # rai-core pins <3.13 -- see pyproject.toml

setup(
    python_requires=PYTHON_REQUIRES,
    packages=["rak_ai"],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
    ],
    scripts=["scripts/agent_node"],
)
