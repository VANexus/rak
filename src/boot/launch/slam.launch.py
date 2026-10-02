"""Full SLAM chain: lidar -> odometry -> slam_toolbox -> /map.

    ros2 launch boot slam.launch.py                    # rf2o (current default)
    ros2 launch boot slam.launch.py odom_source:=none  # identity odom, for testing
    ros2 launch boot slam.launch.py odom_source:=ekf   # once encoders + IMU exist

`odom_source` is the seam. slam_toolbox only ever consumes the
`odom -> base_link` transform and does not care who publishes it, so swapping
rf2o for a robot_localization EKF later is a flag on this command line -- the
slam_toolbox config file is untouched. That is the design; keep the odom
coupling in here and out of config/slam_toolbox.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = get_package_share_directory("boot")

    # Measure this on the hardware: it is the vertical offset from base_link's
    # origin to the lidar's optical centre. A wrong value is a small, silent,
    # permanent error in every map.
    lidar_z = LaunchConfiguration("lidar_z")
    odom_source = LaunchConfiguration("odom_source")

    # Must be a LifecycleNode, not a Node: slam_toolbox's default lifecycle
    # state is `unconfigured`, in which it subscribes to nothing, publishes
    # neither /map nor map->odom, and logs nothing about it. Declared here so
    # the configure event and the activate handler below can both name the
    # *same* action object -- matching by name does not work across actions.
    mapper = LifecycleNode(
        namespace="",
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[os.path.join(share, "config", "slam_toolbox.yaml")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_z", default_value="0.0", description="base_link -> laser_link height [m]"),
            DeclareLaunchArgument(
                "odom_source",
                default_value="rf2o",
                choices=["rf2o", "ekf", "none"],
                description="who publishes odom -> base_link",
            ),
            # The lidar, always.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(share, "launch", "lidar.launch.py")),
            ),
            # base_link -> laser_link. Rigid mount, so a static transform is the
            # whole story -- no URDF, no robot_state_publisher, no joint_states.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="lidar_mount_tf",
                arguments=["0", "0", lidar_z, "0", "0", "0", "base_link", "laser_link"],
            ),
            # Odometry: rf2o from laser scan matching. Publishes /odom and the
            # odom -> base_link transform. odom_topic MUST be overridden --
            # rf2o defaults to /odom_rf2o.
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "base_frame_id": "base_link",
                        "odom_frame_id": "odom",
                        "publish_tf": True,
                        "freq": 10.0,
                        # MUST be empty. Upstream defaults this to
                        # /base_pose_ground_truth -- a Gazebo topic that never
                        # arrives on real hardware -- and LaserCallBack ignores
                        # every scan until GT_pose_initialized flips true
                        # (CLaserOdometry2DNode.cpp:84,192). Leaving the default
                        # makes rf2o silently process nothing at all.
                        "init_pose_from_topic": "",
                    }
                ],
                condition=IfCondition(EqualsSubstitution(odom_source, "rf2o")),
            ),
            # Identity odom for testing the slam_toolbox chain with nothing
            # else in the loop. Quality is poor by design.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="identity_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
                condition=IfCondition(EqualsSubstitution(odom_source, "none")),
            ),
            # The mapper. slam_toolbox is a *LifecycleNode*: left alone it sits
            # in `unconfigured` forever, publishes no map->odom and no /map, and
            # gives no error while doing it. The two events below are the
            # configure -> activate dance that slam_toolbox's own upstream
            # launch does; without them this chain looks "running" and is dead.
            mapper,
            EmitEvent(
                event=ChangeState(
                    lifecycle_node_matcher=matches_action(mapper),
                    transition_id=Transition.TRANSITION_CONFIGURE,
                ),
            ),
            # Activate only once configure has actually landed in `inactive`;
            # firing both at once races and can leave the node configured-but-
            # inactive, which is as silent a failure as never configuring.
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=mapper,
                    start_state="configuring",
                    goal_state="inactive",
                    entities=[
                        EmitEvent(
                            event=ChangeState(
                                lifecycle_node_matcher=matches_action(mapper),
                                transition_id=Transition.TRANSITION_ACTIVATE,
                            )
                        )
                    ],
                )
            ),
        ]
    )
