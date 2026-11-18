"""Full SLAM chain: lidar -> odometry -> slam_toolbox -> /map.

    ros2 launch rak_bringup slam.launch.py                    # rf2o (current default)
    ros2 launch rak_bringup slam.launch.py odom_source:=none  # identity odom, for testing
    ros2 launch rak_bringup slam.launch.py odom_source:=ekf   # 轮式+IMU EKF（2026-11-18 P0-1 收口）

`odom_source` is the seam. slam_toolbox only ever consumes the
`odom -> base_footprint` transform (REP-120) and does not care who publishes it,
so swapping rf2o for a robot_localization EKF later is a flag on this command
line -- the slam_toolbox config file is untouched.

ekf 档的"单一发布者"自证（REP-120，2026-11-18 P0-1）：odom → base_footprint
在一条链里只允许一个发布者。本文件以三种档位各自保证：rf2o 档 = rf2o 自己
发布 TF；ekf 档 = rf2o 仍运行但 publish_tf 被下面的 PythonExpression 强制为
False（只喂 /odom 话题给 EKF），TF 由 ekf_filter_node 发布；none 档 = 只有
identity_odom_tf。因此：不要把本文件与第二个 EKF 实例、
mecanum_drive_controller 的 enable_odom_tf（chassis_controllers.yaml 已固定
false）、或单独再起的 rf2o 叠在一起——两个发布者会让 TF 抖动，slam_toolbox
无告警地漂。localization.launch.py 不在这条链里：它只管 map→odom（rtabmap），
从不发布 odom→base_footprint，也从不启动第二个 EKF，两边注释已写死这层分工。

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
    LogInfo,
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

from rak_orchestration.launch_helpers import managed  # [AI] odom_cov_guard 的受管启动
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
    ekf_params = LaunchConfiguration("ekf_params")
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
                "ekf_params",
                default_value=os.path.join(share, "config", "ekf.yaml"),
                description="robot_localization params file. 默认 config/ekf.yaml"
                            "（odom0=rf2o 全关 P1-2、odom1=轮式 /mecanum_drive_controller/"
                            "odometry 的 x/y/vx、imu0 只取 vyaw）；原地旋转测试裁决通过后"
                            "换 config/ekf_rf2o_yaw_only.yaml 开 rf2o 航向差分。"
                            "ekf 档下 odom→base_footprint 的唯一发布者就是本 EKF"
                            "（rf2o publish_tf 已关，identity_odom_tf 只在 none 档）。",
            ),
            DeclareLaunchArgument(
                "slip_cov_guard", default_value="true", choices=["true", "false"],
                description="L2 轮式 odom 协方差中继（ekf 档才起；"
                            "/mecanum_drive_controller/odometry -> /odom_wheel）",
            ),
            DeclareLaunchArgument(
                "publish_robot_tf", default_value="true",
                description="start robot_state_publisher for the body tree; false if chassis_bringup already does",
            ),
            DeclareLaunchArgument(
                "publish_world_tf", default_value="true",
                description="publish world -> map anchor; false if another node owns it",
            ),
            # The lidar, unless the outer bringup layer already started one.
            DeclareLaunchArgument(
                "start_lidar", default_value="true",
                description="start lidar here; false when the outer layer already did",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sensors, "launch", "lidar.launch.py")),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
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
            # [AI] 2026-11-18 P1-2 坐标系语义（实测 0.453 m 直行）：rf2o 发布的
            # 位姿不自洽——位移方向 +37.9° = -lidar_yaw 的量值（URDF
            # laser_mount_joint = -0.6561 rad），即位移仍处在激光系；绝对航向差
            # 177°，但航向"变化量"是准的。上游无安装旋转参数、twist.linear.y
            # 硬编码 0，launch 层修不了 → EKF 侧显式降级（odom0 全关），裁决
            # 开关见 config/ekf_rf2o_yaw_only.yaml 与
            # scripts/rf2o_frame_audit.py（判据脚本，一条命令出结论）。
            # 这里不改 base_frame_id/odom_frame_id：改成激光系别名只能修"直行"
            # 的位移方向，原地旋转时雷达挂在 (0.11, 0) 处的圆弧项
            # |(I-Rot)·t_m| = 2·0.11·sin45° ≈ 0.16 m（90°）无法用静态偏置表达。
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
            # [AI] EKF: ekf 档下接管 odom->base_footprint（唯一发布者，团队 ADR 规矩）。
            # 融合（2026-11-18 P0-1 收口，详见 config/ekf.yaml 注释）：
            #   odom0 = /odom（rf2o；位姿按 P1-2 全关——位移 +37.9° 是激光系泄漏）
            #   odom1 = /mecanum_drive_controller/odometry（轮式 x/y/vx，yaw/vyaw 关）
            #   imu0  = /imu/data_raw（只取 vyaw）
            # 单一发布者自证：ekf 档下 rf2o 的 publish_tf 由上面 PythonExpression
            # 强制为 False，identity_odom_tf 只在 none 档 —— 全链只有本节点发布
            # odom→base_footprint。不要再叠加：第二个 EKF 实例、rf2o 单独启动、
            # 或 mecanum_drive_controller enable_odom_tf（已固定 false）。
            # 启动时下面的 LogInfo 会把这条不变量打在屏幕上，便于现场核对。
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[LaunchConfiguration("ekf_params")],
                condition=IfCondition(EqualsSubstitution(odom_source, "ekf")),
            ),
            # [AI] L2 协方差中继（§7.2 / P1-1）：轮式 odom 的唯一来源。
            # ekf.yaml 的 odom1 订阅 /odom_wheel，所以本节点**必须**与 EKF 同生：
            # 它由本 launch 拥有（单一生命周期拥有者），nav2.launch.py 不再起第二份
            # （那里只保留 slip_cov_guard 透传开关，避免双拥有者）。
            # 打滑期间把 pose(x,y)/twist(vx,vy) 协方差 ×cov_factor，信号来自
            # slip_watch 的 /slip_watch/slip_suspected；信号缺失时 fail-open 原样转发。
            *managed(
                package="rak_metrics",
                executable="odom_cov_guard",
                name="odom_cov_guard",
                output="screen",
                autostart=True,
                condition=IfCondition(PythonExpression([
                    "'", odom_source, "' == 'ekf' and '",
                    LaunchConfiguration("slip_cov_guard"), "' == 'true'"])),
            ),
            LogInfo(
                condition=IfCondition(EqualsSubstitution(odom_source, "ekf")),
                msg=["[slam.launch] odom_source=ekf: odom->base_footprint 唯一发布者 = ",
                     "ekf_filter_node, params=", ekf_params,
                     "; rf2o 仅喂 /odom 话题, publish_tf 已关; ",
                     "identity_odom_tf 仅 none 档。勿再叠加第二个 odom TF 发布者。"],
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
