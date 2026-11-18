"""Fixed-map visual localization: RTAB-Map localization mode + AprilTag anchors.

v1.0's runtime has no lidar (`docs/design/vision_nav_v1.md` §3). This entry brings up the
localization half only: the static map and the Nav2 stack stay in `nav2.launch.py
sensors:=camera`, the chassis/controllers stay in `chassis_bringup.launch.py`.

TF ownership (ADR-0002, one publisher per frame):
  map -> odom        : rtabmap (localization mode)  <- this launch
  odom -> base_footprint : robot_localization EKF   <- slam.launch.py odom_source:=ekf
  everything else    : robot_state_publisher (URDF)

分工与互斥（2026-11-18 P0-1 收口，写死在这里避免现场叠错）：
- 本文件与 slam.launch.py 是 map→odom 的**二选一替代**：rtabmap（本文件）或
  slam_toolbox（slam.launch.py），同时起 = 两个 map→odom 发布者，地图帧
  非确定漂移。二者永不同时跑。
- odom→base_footprint 只有 slam.launch.py odom_source:=ekf 里的 ekf_filter_node
  一个发布者。本文件**从不**启动 EKF、从不发布 odom→base_footprint——它只把
  /odometry/filtered（EKF 的输出）当重定位先验消费。所以用本入口前必须先把
  `ros2 launch rak_bringup slam.launch.py odom_source:=ekf` 跑起来。
- 禁止再起第二个 EKF 实例或别的 odom TF 发布者（identity_odom_tf、控制器
  enable_odom_tf 等）；单一发布者不变量在 slam.launch.py 的文档串与启动
  LogInfo 里也有写明。

Camera model: v1.0 is MONOCULAR by design (vision_nav_v1.md §5A). Metric scale in
localization mode comes from the EKF wheel/IMU odometry as the pose prior, and the
absolute anchor comes from AprilTag -- a monocular camera does not need stereo
calibration. What it DOES need is intrinsics (fx, fy, cx, cy), otherwise neither the
tag PnP nor rtabmap can produce a metric pose. Pass `camera_model:=stereo` only when
the stereo pair is calibrated and mounted on the base (the eye-in-hand pair belongs to
the arm, not here).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("rak_bringup")
    rtabmap_launch = os.path.join(
        get_package_share_directory("rtabmap_launch"), "launch", "rtabmap.launch.py"
    )
    rtabmap_cfg = os.path.join(bringup_share, "config", "rtabmap_localization.yaml")
    apriltag_cfg = os.path.join(bringup_share, "config", "apriltag.yaml")

    database_path = LaunchConfiguration("database_path")
    frame_id = LaunchConfiguration("frame_id")
    odom_topic = LaunchConfiguration("odom_topic")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_apriltag = LaunchConfiguration("use_apriltag")
    camera_model = LaunchConfiguration("camera_model")
    apriltag_image = LaunchConfiguration("apriltag_image")
    apriltag_camera_info = LaunchConfiguration("apriltag_camera_info")

    return LaunchDescription([
        DeclareLaunchArgument(
            "database_path",
            default_value=os.path.expanduser("~/.ros/rtabmap.db"),
            description="RTAB-Map database built once (and only once) offline.",
        ),
        DeclareLaunchArgument(
            "camera_model",
            default_value="mono",
            choices=["mono", "stereo"],
            description="mono: front camera intrinsics + EKF odometry prior (v1.0 default). "
                        "stereo: only when a calibrated stereo pair is mounted on the base.",
        ),
        DeclareLaunchArgument("frame_id", default_value="base_footprint"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odometry/filtered",
            description="EKF 输出（/odometry/filtered），作为重定位位姿先验。"
                        "注意：本入口不起 EKF；odom→base_footprint 必须已由"
                        " slam.launch.py odom_source:=ekf 提供。",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "use_apriltag",
            default_value="true",
            description="Run apriltag_ros. Without tags there is no absolute anchor.",
        ),
        DeclareLaunchArgument("apriltag_image", default_value="camera/image_raw"),
        DeclareLaunchArgument("apriltag_camera_info", default_value="camera/camera_info"),
        DeclareLaunchArgument(
            "rgb_topic",
            default_value="camera/image_raw",
            description="camera_model:=mono input.",
        ),
        DeclareLaunchArgument(
            "camera_info_topic", default_value="camera/camera_info"
        ),

        LogInfo(msg=(
            "visual localization: rtabmap in LOCALIZATION mode (no new map nodes). "
            "map->odom here; odom->base_footprint must come from the EKF. "
            "Do not run alongside slam.launch.py."
        )),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rtabmap_launch),
            launch_arguments={
                "localization": "true",
                "database_path": database_path,
                "frame_id": frame_id,
                "odom_topic": odom_topic,
                # mono: EKF odometry is the scale prior, AprilTag the absolute anchor.
                "stereo": EqualsSubstitution(camera_model, "stereo"),
                "subscribe_rgb": EqualsSubstitution(camera_model, "mono"),
                "subscribe_depth": "false",
                "subscribe_rgbd": "false",
                "rgb_topic": LaunchConfiguration("rgb_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                "visual_odometry": "false",
                "approx_sync": "true",
                "queue_size": "10",
                "sync_queue_size": "10",
                "cfg": rtabmap_cfg,
                "rtabmap_viz": "false",
                "rviz": "false",
                "use_sim_time": use_sim_time,
                "left_image_topic": "eye/left/image_raw",
                "right_image_topic": "eye/right/image_raw",
                "tag_topic": "/apriltag/detections",
            }.items(),
        ),

        Node(
            package="apriltag_ros",
            executable="apriltag_node",
            name="apriltag",
            namespace="apriltag",
            output="screen",
            parameters=[apriltag_cfg, {"use_sim_time": use_sim_time}],
            remappings=[
                ("image_rect", apriltag_image),
                ("camera_info", apriltag_camera_info),
            ],
            condition=IfCondition(EqualsSubstitution(use_apriltag, "true")),
        ),
    ])
