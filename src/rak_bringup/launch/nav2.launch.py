"""SLAM + omnidirectional Nav2 with optional local motion-test adapters.

Velocity path: MPPI / behavior -> velocity smoother -> collision monitor ->
the selected final-Twist adapter on /cmd_vel. ``none`` leaves the endpoint to
an outer bringup layer, which can compose a hardware driver without reversing
the package dependency direction:

  - sim        cmd_vel_sim      (sim-only /base/telemetry mirror)
  - stub       cmd_vel_stub     (Twist relay -> /cmd_vel_raw)
  - none       no adapter in this launch
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node

#: The servers `navigate_to_pose_w_replanning_and_recovery.xml` actually resolves.
#: `nav2_bringup`'s navigation_launch.py always starts ten; the four dropped here were
#: measured at ~19% of a core on an otherwise idle stack, doing nothing this robot asks
#: for -- `smoother_server` (the default BT never calls it), `route_server` and
#: `docking_server` (no route or dock use case), `waypoint_follower` (the task executor
#: drives NavigateToPose, not NavigateThroughPoses).
_NAV2_SERVERS = (
    ("nav2_controller", "controller_server", "controller_server", True),
    ("nav2_planner", "planner_server", "planner_server", False),
    ("nav2_behaviors", "behavior_server", "behavior_server", True),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator", False),
    ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother", False),
    ("nav2_collision_monitor", "collision_monitor", "collision_monitor", False),
)


def _navigation_servers(params_file, use_sim_time):
    """Start the lifecycle servers and their manager directly, minus the dead weight."""
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = []
    for package, executable, name, remap_cmd_vel in _NAV2_SERVERS:
        nodes.append(
            Node(
                package=package,
                executable=executable,
                name=name,
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                remappings=remappings + ([("cmd_vel", "cmd_vel_nav")] if remap_cmd_vel else []),
            )
        )
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"autostart": True},
                {"use_sim_time": use_sim_time},
                {"node_names": [name for _, _, name, _ in _NAV2_SERVERS]},
            ],
        )
    )
    return nodes


def generate_launch_description():
    nav_share = get_package_share_directory("rak_bringup")
    nav2_params = os.path.join(nav_share, "config", "nav2_params.yaml")
    backend = LaunchConfiguration("motion_backend")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "motion_backend",
            default_value="sim",
            choices=["sim", "stub", "none"],
            description="Final /cmd_vel adapter; none leaves it to an outer bringup.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_z", default_value="0.0"),
        DeclareLaunchArgument("odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav_share, "launch", "slam.launch.py")),
            launch_arguments={
                "lidar_z": LaunchConfiguration("lidar_z"),
                "odom_source": LaunchConfiguration("odom_source"),
            }.items(),
        ),
        Node(
            package="rak_motion_stub",
            executable="cmd_vel_sim",
            name="cmd_vel_sim",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "telemetry_topic": "/base/telemetry",
            }],
            condition=IfCondition(EqualsSubstitution(backend, "sim")),
        ),
        Node(
            package="rak_motion_stub",
            executable="cmd_vel_stub",
            name="cmd_vel_stub",
            output="screen",
            parameters=[{
                "cmd_vel_topic": "/cmd_vel",
                "raw_topic": "/cmd_vel_raw",
                "log_every_n": 5,
            }],
            condition=IfCondition(EqualsSubstitution(backend, "stub")),
        ),
        *_navigation_servers(nav2_params, use_sim_time),
    ])
