"""rak's decision layer: System 1 (fast, local) + System 2 (slow, RAI).

This code is installed and run as the ROS 2 package `rak_agent` via colcon.
The same policy in `system1.resolve()` is used by the node and local tests:

    PYTHONPATH=src/rak_agent python3 -m rak_agent.loop ...  # scripted body, for tests and episodes
    ros2 run rak_agent agent_node   # live perception, for the robot

System 1 runs in whichever process imports it -- never behind a topic or a
service. Only perception and decisions cross the wire.
"""
