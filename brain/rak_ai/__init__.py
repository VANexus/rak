"""rak's decision layer: System 1 (fast, local) + System 2 (slow, RAI).

A ROS 2 package (`brain`, ament_python) as well as an ordinary Python package:
`pip install -e .` still works, and so does `colcon build`. There are two entry
points into the same code and they share the tick policy in `system1.resolve()`:

    python -m rak_ai.loop ...   # scripted body, for tests and episodes
    ros2 run brain brain_node   # live perception, for the robot

System 1 runs in whichever process imports it -- never behind a topic or a
service. Only perception and decisions cross the wire.
"""
