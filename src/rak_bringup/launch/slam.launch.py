"""Full SLAM chain: lidar -> odometry -> slam_toolbox -> /map.

    ros2 launch rak_bringup slam.launch.py                    # rf2o (current default)
    ros2 launch rak_bringup slam.launch.py odom_source:=none  # identity odom, for testing
    ros2 launch rak_bringup slam.launch.py odom_source:=ekf   # once encoders + IMU exist

`odom_source` is the seam. slam_toolbox only ever consumes the
`odom -> base_footprint` transform (REP-120) and does not care who publishes it,
so swapping rf2o for a robot_localization EKF later is a flag on this command
line -- the slam_toolbox config file is untouched.

Frame tree this launch guarantees (the single static-geometry authority):

    world -> map -> odom -> base_footprint -> base_link -> {laser,mic,camera,imu,arm}

The robot body tree comes from robot_state_publisher loading the whole-robot
xacro (rak_description/urdf/rak_chassis.urdf.xacro); the sensor mounts are
fixed joints there, NOT ad-hoc static_transform_publishers here -- one publisher
per frame. `world` is the unbounded global anchor for the open world; it is a
static identity onto `map` until a global-localization module exists.

    publish_robot_tf:=false   # set when chassis_bringup.launch.py already runs
                             # robot_state_publisher, to avoid a second
                             # base_footprint -> base_link publisher.
    publish_world_tf:=false   # set when another node owns world -> map.
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
from launch.substitutions import (
    Command,
    EqualsSubstitution,
    LaunchConfiguration,
    PythonExpression,  # [AI] ekf 档条件与 publish_tf 布尔
)
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue  # [AI] 布尔参数
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

#: whole-robot static skeleton (REP-120 base frames + wheels + sensor/arm mounts)
DESCRIPTION_URDF = os.path.join(
    get_package_share_directory("rak_description"), "urdf", "rak_chassis.urdf.xacro"
)


def generate_launch_description():
    share = get_package_share_directory("rak_bringup")
    sensors = get_package_share_directory("rak_bringup")

    # Measure this on the hardware: it is the vertical offset from base_link's
    # origin to the lidar's optical centre, passed through to the URDF. A wrong
    # value is a small, silent, permanent error in every map.
    lidar_z = LaunchConfiguration("lidar_z")
    odom_source = LaunchConfiguration("odom_source")
    publish_robot_tf = LaunchConfiguration("publish_robot_tf")
    publish_world_tf = LaunchConfiguration("publish_world_tf")

    robot_description = ParameterValue(
        Command(["xacro ", DESCRIPTION_URDF, " lidar_z:=", lidar_z]),
        value_type=str,
    )

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
        parameters=[
            os.path.join(share, "config", "slam_toolbox.yaml"),
            # 地图落盘位置：默认 $HOME/.ros/rak/maps（ROS 惯例位置），可被
            # map_file:=... 覆盖。写进 yaml 就等于把家目录钉死在源码里。
            {"map_file_name": ParameterValue(
                LaunchConfiguration("map_file"), value_type=str)},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file",
                default_value=os.path.join(
                    os.path.expanduser("~"), ".ros", "rak", "maps", "rak_room"),
                description="lifelong 位姿图落盘路径（默认 $HOME/.ros/rak/maps/rak_room）",
            ),
            DeclareLaunchArgument("lidar_z", default_value="0.1525", description="base_link -> laser_link height [m] (2026-10-14 实测)"),
            DeclareLaunchArgument(
                "odom_source",
                default_value="rf2o",
                choices=["rf2o", "ekf", "none"],
                description="who publishes odom -> base_footprint",
            ),
            DeclareLaunchArgument(
                "publish_robot_tf", default_value="true",
                description="start robot_state_publisher for the body tree; false if chassis_bringup already does",
            ),
            DeclareLaunchArgument(
                "publish_world_tf", default_value="true",
                description="publish world -> map anchor; false if another node owns it",
            ),
            # The lidar, always.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sensors, "launch", "lidar.launch.py")),
            ),
            # Body tree (base_footprint -> base_link -> wheels/sensors/arm mount).
            # One authority for the whole static skeleton; the sensor mounts live
            # in the URDF, not as separate static_transform_publishers here.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(publish_robot_tf),
            ),
            # world -> map: unbounded global anchor for the open world. Static
            # identity until a global-localization module replaces it; the frame
            # itself keeps map from having to double as the global root.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_map_tf",
                arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
                condition=IfCondition(publish_world_tf),
            ),
            # Odometry: rf2o from laser scan matching. Publishes /odom and the
            # odom -> base_footprint transform. odom_topic MUST be overridden --
            # rf2o defaults to /odom_rf2o.
            # [AI] 2026-10-09: odom_source:=ekf 档下 rf2o 仍然运行(喂 EKF 的
            # odom0), 但 publish_tf=False —— odom->base_footprint 由 EKF 独占发布。
            Node(
                package="rf2o_laser_odometry",
                executable="rf2o_laser_odometry_node",
                name="rf2o_laser_odometry",
                output="screen",
                parameters=[
                    {
                        "laser_scan_topic": "/scan",
                        "odom_topic": "/odom",
                        "base_frame_id": "base_footprint",
                        "odom_frame_id": "odom",
                        # [AI] True=rf2o 自发布 TF(rf2o 档); False=仅喂 /odom(ekf 档)
                        "publish_tf": ParameterValue(
                            PythonExpression(["'", odom_source, "' == 'rf2o'"]),
                            value_type=bool,
                        ),
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
                # [AI] rf2o 档与 ekf 档都要跑 (ekf 档下它是融合输入源)
                condition=IfCondition(
                    PythonExpression(["'", odom_source, "' in ('rf2o', 'ekf')"])
                ),
            ),
            # [AI] EKF: ekf 档下接管 odom->base_footprint(唯一发布者, 团队 ADR 规矩);
            # 融合 odom0=/odom(rf2o 激光里程计) + imu0=/imu/data_raw(LSM6DSRTR 航向速率)
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[os.path.join(share, "config", "ekf.yaml")],
                condition=IfCondition(EqualsSubstitution(odom_source, "ekf")),
            ),
            # Identity odom for testing the slam_toolbox chain with nothing
            # else in the loop. Quality is poor by design.
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="identity_odom_tf",
                arguments=["0", "0", "0", "0", "0", "0", "odom", "base_footprint"],
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
