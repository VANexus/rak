"""Run the brain against live (or simulated) perception.

    ros2 launch rak_bringup agent.launch.py                       # stub decider + sim
    ros2 launch rak_bringup agent.launch.py sim:=false            # wait for real
                                                           # /perception/tracked_objects
    ros2 launch rak_bringup agent.launch.py supervised:=true      # 交给 system_manager

`sim` defaults to true because nothing publishes TrackedObjects yet: the
simulator emits the scripted body on the real perception topic, so the whole
path -- message in, SceneState, guard/gate/resolve, Decision out -- is the same
code that will run when perception lands. Set `sim:=false` and the node just
waits, publishing `inspect` once `max_state_age` expires.

System 1 runs inside the node process. Only perception and decisions cross it.

生命周期（ADR-0020 §5）：`/brain` 是 `ManagedNode`，TensorRT 引擎在 `on_configure`
加载。走 `managed(...)` 而不是 `Node(...)` —— 后者会让进程停在 `unconfigured`：
订阅不建、话题不发、也不报错。`supervised:=true` 时本文件不发转移，等编排层；
默认 false 自启，单独跑这个 launch 排障时不会出现"进程在但不干活"。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

from rak_orchestration.launch_helpers import managed


def _arg(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default)


def _param(name: str, value_type):
    return name, ParameterValue(LaunchConfiguration(name), value_type=value_type)


def generate_launch_description() -> LaunchDescription:
    parameters = dict(
        [
            _param("decider", str),  # stub | tensorrt
            _param("consultant", str),  # stub | rai | a vendor name
            _param("scenario", str),  # approach | cross | clear
            _param("sim", bool),
            _param("rate", float),  # sim + watchdog Hz
            _param("force_s2", int),  # escalate from this tick on; -1 = only on gate
            _param("max_state_age", float),  # seconds of silence before stale
            _param("scene", str),  # SceneState.scene -- see node.py
            _param("flags", str),  # comma separated SceneState.flags
            _param("verbose", bool),
            # "" -> <repo>/data/episodes (ADR-0013) (the CLI's corpus, resolved even when
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
            _arg("supervised", "false"),
            *managed(
                package="rak_agent",
                executable="agent_node",
                name="brain",
                output="screen",
                parameters=[parameters],
            ),
        ]
    )
