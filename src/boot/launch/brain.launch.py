"""Run the brain against live (or simulated) perception.

    ros2 launch boot brain.launch.py                       # stub decider + sim
    ros2 launch boot brain.launch.py decider:=laya sim:=true
    ros2 launch boot brain.launch.py sim:=false            # wait for real
                                                           # /perception/tracked_objects

`sim` defaults to true because nothing publishes TrackedObjects yet: the
simulator emits the scripted body on the real perception topic, so the whole
path -- message in, SceneState, guard/gate/resolve, Decision out -- is the same
code that will run when perception lands. Set `sim:=false` and the node just
waits, publishing `inspect` once `max_state_age` expires.

System 1 runs inside the node process. Only perception and decisions cross it.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _arg(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default)


def _param(name: str, value_type):
    return name, ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    parameters = dict(
        [
            _param("decider", str),  # stub | laya
            _param("consultant", str),  # stub | rai | a vendor name
            _param("scenario", str),  # approach | cross | clear
            _param("sim", bool),
            _param("rate", float),  # sim + watchdog Hz
            _param("force_s2", int),  # escalate from this tick on; -1 = only on gate
            _param("max_state_age", float),  # seconds of silence before stale
            _param("scene", str),  # SceneState.scene -- see node.py
            _param("flags", str),  # comma separated SceneState.flags
            _param("verbose", bool),
            # "" -> brain/data/episodes (the CLI's corpus, resolved even when
            # this runs from <ws>/install); set it to put ROS runs elsewhere.
            _param("out", str),
            _param("episode_ticks", int),  # 0 = never roll to a new file
        ]
    )

    return LaunchDescription(
        [
            _arg("decider", "stub"),
            _arg("consultant", "stub"),
            _arg("scenario", "approach"),
            _arg("sim", "true"),
            _arg("rate", "10.0"),
            _arg("force_s2", "-1"),
            _arg("max_state_age", "1.0"),
            _arg("scene", "open_floor"),
            _arg("flags", ""),
            _arg("verbose", "true"),
            _arg("out", ""),
            _arg("episode_ticks", "10000"),
            Node(
                package="brain",
                executable="brain_node",
                name="brain",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
