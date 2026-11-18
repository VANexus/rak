"""Unified sensor bringup: every sensor stream + one RViz view.

    ros2 launch rak_bringup sensors.launch.py                      # all sensors, one RViz
    ros2 launch rak_bringup sensors.launch.py camera:=astra        # other depth camera
    ros2 launch rak_bringup sensors.launch.py chassis:=true        # + wheel odometry
    ros2 launch rak_bringup sensors.launch.py slam:=true           # + map/odom TF
    ros2 launch rak_bringup sensors.launch.py rviz:=false          # headless

What lands in RViz (config: config/sensors_rviz.rviz)
    lidar      /scan (LaserScan) + /lslidar_point_cloud (PointCloud2)
    depth cam  /camera/depth/image_raw, /camera/color/image_raw (Gemini only, the
               Astra S is depth-only), /camera/depth/points (PointCloud2, built on
               the robot by depth_image_proc so both backends share one contract)
    IMU        /imu/data_raw (rviz_imu_plugin, LSM6DSRTR @1kHz, frame imu_link)
               /camera/gyro_accel/sample (Gemini 335L @200 Hz, frame
               camera_accel_gyro_optical_frame, accel arrows -- cross-check only)
    mic array  /wake_doa (Marker) -- live bearing arrow + fading event trail
               (repainted continuously, see rak_speech/wake_doa_viz);
               audio itself has no RViz display, the DOA angle is its picture.
    wheel odom /mecanum_drive_controller/odometry (Odometry arrows, needs chassis:=true)

Ownership rules (ADR-0015: one publisher per frame, one publisher per topic)
    body tree (base_footprint -> base_link -> sensors/arm) : robot_state_publisher here,
        unless chassis_bringup already runs one -> publish_robot_tf:=false
    camera optical frames : the camera driver's job (orbbec publish_tf:=true), except
        camera:=astra whose driver does not broadcast TF -> this launch adds it
    IMU : two sources, two topics, no overlap. /imu/data_raw is the EKF/heading
        authority (1 kHz, survives a camera swap); /camera/gyro_accel/sample is
        the camera's own 200 Hz IMU for cross-check and visualisation. Never remap
        one onto the other -- that would put two publishers on one topic.
    odom -> base_footprint : rf2o or EKF (slam.launch.py), never the wheel controller
                             (chassis_controllers.yaml sets enable_odom_tf: false)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rak_orchestration.launch_helpers import managed

#: REP-103 optical frame: z forward, x right, y down.
_OPTICAL_RPY = ("-1.57079632679", "0.0", "-1.57079632679")


def _camera_with_cloud(context, *args, **kwargs):
    """Exactly one depth-camera backend, plus a uniform point cloud on top.

    The backends are deliberately not interchangeable at this layer: Orbbec
    broadcasts its own camera optical frames, the in-house astra driver does not,
    so only the astra path needs the static TF that keeps the cloud in the tree.
    """
    backend = LaunchConfiguration("camera").perform(context)
    bringup_share = get_package_share_directory("rak_bringup")
    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "depth_camera.launch.py")
            ),
            launch_arguments={
                "camera": backend,
                # 跨机 RViz 时传 depth_max_width:=160（见 astra.launch.py 注释）。
                "depth_max_width": LaunchConfiguration("depth_max_width").perform(context),
                "gemini_profile": LaunchConfiguration("camera_profile").perform(context),
                "gemini_ir": LaunchConfiguration("camera_ir").perform(context),
            }.items(),
        ),
        # depth_image_proc: depth image + camera_info -> PointCloud2. Chosen over the
        # SDK's own cloud so both backends honour the same topic contract.
        Node(
            package="depth_image_proc",
            executable="point_cloud_xyz_node",
            name="depth_point_cloud",
            # [AI] 2026-11-15: depth_image_proc 的输入话题是 image_rect/camera_info
            # （非私有、不带 depth 前缀）。原重映射写 "depth/image_rect"，对根命名
            # 空间下的节点不生效 → 点云永远为空（实测 node info 订阅 /image_rect）。
            # 这里直接用节点真实话题名做重映射。
            output="screen",
            remappings=[
                ("image_rect", "/camera/depth/image_raw"),
                ("camera_info", "/camera/depth/camera_info"),
                ("points", "/camera/depth/points"),
            ],
        ),
    ]
    if backend == "astra":
        for child in ("camera_color_frame", "camera_depth_frame"):
            actions.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name=f"{child}_tf",
                    # Jazzy dropped the positional form ("roll pitch yaw parent child"
                    # now dies with "Extra unparsed arguments"); named flags only.
                    arguments=[
                        "--roll", _OPTICAL_RPY[0],
                        "--pitch", _OPTICAL_RPY[1],
                        "--yaw", _OPTICAL_RPY[2],
                        "--frame-id", "camera_link",
                        "--child-frame-id", child,
                    ],
                    output="screen",
                )
            )
    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("rak_bringup")

    # Lidar comes from slam.launch.py when that chain is included; starting it twice
    # would fight over one CP2102 port and double-publish /scan.
    lidar_standalone = IfCondition(
        PythonExpression([
            "'", LaunchConfiguration("lidar"), "' == 'true' and '",
            LaunchConfiguration("slam"), "' != 'true'",
        ])
    )

    return LaunchDescription([
        # ---- frame authority -------------------------------------------------
        DeclareLaunchArgument(
            "publish_robot_tf", default_value="true",
            description="run robot_state_publisher for the body tree; "
                        "false if chassis_bringup.launch.py already runs one",
        ),
        DeclareLaunchArgument(
            "urdf",
            default_value=os.path.join(
                get_package_share_directory("rak_description"),
                "urdf", "rak_chassis.urdf.xacro",
            ),
            description="chassis URDF (xacro) -- geometry single source per ADR-0015",
        ),

        # ---- feature switches ------------------------------------------------
        DeclareLaunchArgument(
            "lidar", default_value="true",
            description="start the lslidar driver (ignored when slam:=true)",
        ),
        DeclareLaunchArgument(
            "camera_on", default_value="true",
            description="start a depth camera and build /camera/depth/points",
        ),
        DeclareLaunchArgument(
            "camera", default_value="gemini335l", choices=["gemini335l", "astra"],
            description="depth camera backend (ADR-0014: exactly one)",
        ),
        DeclareLaunchArgument(
            "depth_max_width", default_value="320",
            description="astra depth width cap (0 = no cap); 160 for cross-host RViz "
                        "over WiFi (image + cloud bandwidth)",
        ),
        DeclareLaunchArgument(
            "camera_profile", default_value="wifi", choices=["wifi", "full"],
            description="gemini335l only: wifi = 424x240 depth@10 + color@5 "
                        "(cross-host RViz), full = 1280x800@30 (local display/cable)",
        ),
        DeclareLaunchArgument(
            "camera_ir", default_value="none",
            choices=["none", "left", "right", "both"],
            description="gemini335l only: publish left/right IR images "
                        "(/camera/ir/left/image_raw, /camera/ir/right/image_raw)",
        ),
        DeclareLaunchArgument(
            "imu", default_value="true",
            description="start the Jetson-direct LSM6DSRTR I2C node (/imu/data_raw)",
        ),
        DeclareLaunchArgument(
            "mic", default_value="true",
            description="start the six-mic array speech pipeline",
        ),
        DeclareLaunchArgument(
            "chassis", default_value="false",
            description="include chassis_bringup (opt-in: talks to real hardware, "
                        "supplies wheel odometry)",
        ),
        DeclareLaunchArgument(
            "allow_motion", default_value="false",
            description="passed to chassis_bringup only; false = DISARMED",
        ),
        DeclareLaunchArgument(
            "slam", default_value="false",
            description="include slam.launch.py (map + odom->base_footprint TF)",
        ),
        DeclareLaunchArgument(
            "odom_source", default_value="rf2o", choices=["rf2o", "ekf", "none"],
            description="passed to slam.launch.py only",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="start RViz with config/sensors_rviz.rviz",
        ),
        # ADR-0020: wake_doa_viz 是受管节点。默认 false = 本文件自己
        # configure→activate（单跑 sensors 排障）；true = 转移交给 system_manager。
        DeclareLaunchArgument(
            "supervised", default_value="false",
            description="true = 转移交给 system_manager，本文件不自启受管节点",
        ),

        # ---- body tree (one publisher) ---------------------------------------
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(
                    Command(["xacro ", LaunchConfiguration("urdf")]),
                    value_type=str,
                ),
            }],
            condition=IfCondition(LaunchConfiguration("publish_robot_tf")),
        ),

        # ---- lidar -----------------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "lidar.launch.py")
            ),
            condition=lidar_standalone,
        ),

        # ---- depth camera + point cloud --------------------------------------
        OpaqueFunction(function=_camera_with_cloud,
                       condition=IfCondition(LaunchConfiguration("camera_on"))),

        # ---- Jetson-direct IMU ----------------------------------------------
        # [AI] 2026-10-26: 改用 I2C。SPI 路径在本机走不通——LSM6DSR 在上电时按 CS
        # 电平选接口，而 Jetson 的 SPI CS 空闲为高，芯片因此锁定 I2C 模式；
        # I2C 实测 994-1000 Hz、错误率约 2.5 万分之一（详见 i2c_imu.py 头注）。
        # axis_map 默认值由驱动提供（-z,-x,y，本机安装朝向实测校准），这里显式
        # 写出只是为了让 launch 自解释；换模块/重装后必须重测三轴。
        # 受管节点（ADR-0020 / ADR-0023）：设备在 configure 打开，1 kHz 采样在 activate
        # 才开始；时间戳取自采样时刻。supervised:=true 时由 system_manager 驱动。
        GroupAction(actions=managed(
            package="rak_imu_driver",
            executable="i2c_imu",
            name="i2c_imu",
            output="screen",
            parameters=[{
                "device": "/dev/i2c-7",
                "address": 0x6B,
                "rate_hz": 1000.0,
                "frame_id": "imu_link",
                "topic": "/imu/data_raw",
                "axis_map": "-z,-x,y",
                # [AI] 陀螺零偏（rad/s），在 axis_map 之后施加。
                # 2026-11-18 重标定（**静置 60 s，35786 帧，596.5 Hz，az=9.7182 确认
                # 传感器未冻结**）：gx=-0.0079 °/s、gy=+0.0108 °/s、gz=+0.0219 °/s。
                # 标准误 ~0.001 °/s。不补的后果：仅 Z 轴 +0.022 °/s 就够在 10 min
                # 累积 +13.2°，EKF 的 vyaw 直接吃这个误差，地图整体转歪。
                # robot_localization 的 EKF **不会**自己估计零偏，只能在这里补。
                # 重测（车必须完全静止，且先看 i2c_imu 日志的 freeze_reinit 不为异常增长）：
                #   ros2 run rak_imu_driver i2c_imu &  # configure+activate
                #   python3 /tmp/bias60.py             # 60 s 静置，输出即总零偏
                # 旧值（2026-10-29）：-0.002902,-0.006872,-0.002291 —— Z 轴差 0.0027 rad/s
                # ≈ 0.155 °/s，10 min 就是 56°，这是之前"航向不可信"的主要来源。
                "gyro_bias": "0.001186,-0.007734,-0.003310",
            }],
        ), condition=IfCondition(LaunchConfiguration("imu"))),

        # ---- microphone array ------------------------------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "speech.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("mic")),
        ),
        # The board only reports a bearing together with a wake event, and audio has
        # no RViz display -- wake_doa_viz holds the latest bearing and repaints it
        # (plus a fading trail of recent wakes) so the view stays live.
        # Managed node (ADR-0020 §5)：`managed()` 返回"节点 + 两个事件处理器"一组动作，
        # 所以 mic 开关落在 GroupAction 上，整组一起起/停。
        GroupAction(
            actions=managed(
                package="rak_speech",
                executable="wake_doa_viz",
                name="wake_doa_viz",
                output="screen",
            ),
            condition=IfCondition(LaunchConfiguration("mic")),
        ),

        # ---- wheel odometry (opt-in, real hardware) --------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "chassis_bringup.launch.py")
            ),
            launch_arguments={
                "allow_motion": LaunchConfiguration("allow_motion"),
            }.items(),
            condition=IfCondition(LaunchConfiguration("chassis")),
        ),

        # ---- SLAM chain: owns map, and odom -> base_footprint ----------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_share, "launch", "slam.launch.py")
            ),
            launch_arguments={
                "odom_source": LaunchConfiguration("odom_source"),
                # this launch already owns the body tree
                "publish_robot_tf": "false",
            }.items(),
            condition=IfCondition(LaunchConfiguration("slam")),
        ),

        # ---- single RViz -----------------------------------------------------
        Node(
            package="rviz2",
            executable="rviz2",
            name="sensors_rviz",
            output="screen",
            arguments=[
                "-d",
                os.path.join(bringup_share, "config", "sensors_rviz.rviz"),
            ],
            condition=IfCondition(LaunchConfiguration("rviz")),
        ),
    ])
