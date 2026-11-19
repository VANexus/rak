"""纯视觉寻路，按角色拆分驱动端与逻辑端（2026-11-19 用户口径）。

    # 工作站（逻辑端）：只跑 trace + 控制 + RViz，订阅板子上的 mask
    ros2 launch rak_bringup road_follow.launch.py role:=logic

    # Jetson（驱动端）：相机 + 分割，headless
    ros2 launch rak_bringup road_follow.launch.py role:=driver

    # 单机自测（两端同一台）
    ros2 launch rak_bringup road_follow.launch.py role:=both rviz:=false

为什么拆两半：分割要吃 GPU 和相机，只能在板上；trace/控制是纯 numpy，放工作站
便于热改和看 RViz。两边只通过标准话题耦合——`/road_perception/road_mask`（板上
发布，本机订阅）和 `/cmd_vel` + `/mecanum_drive_controller/reference`（本机发布，
板上消费）。跨机要两端 `rmw_cyclonedds_cpp` + 同一 `ROS_DOMAIN_ID` +
`CYCLONEDDS_URI`（见 docs/reference/perf/cross_host_dds.md）。

运动许可两道门，都为 false 时整条链只读：
  * `enabled`           —— road_follow_node 是否解算并发布
  * `allow_motion`      —— 是否把命令写进 `mecanum_drive_controller` 的 reference
真机动车前还需要 `chassis_bringup.launch.py allow_motion:=true`，且单写者纪律
（ADR-0011）要求 Nav2 / rak_wbc / chassis_tui 全部不在跑。

`camera_device` 默认会被 `_pick_front_camera()` 按 sysfs 名字覆盖：这块板的
`/dev/video0` 是 Orbbec 深度流，不是前视 UVC。要强制指定就显式传。
"""

import glob
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    from ament_index_python.packages import get_package_share_directory

    _CONFIG_DIR = os.path.join(get_package_share_directory("rak_bringup"), "config")
except Exception:  # source tree without a local rak_bringup install
    _CONFIG_DIR = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "config")

_DEFAULT_RVIZ = os.path.join(_CONFIG_DIR, "road_follow.rviz")
_DEFAULT_CAMERA_INFO = os.path.join(_CONFIG_DIR, "camera_info", "front_camera_640x480.yaml")


def _arg(name: str, default: str) -> DeclareLaunchArgument:
    return DeclareLaunchArgument(name, default_value=default)


def _param(name: str, value_type):
    return name, ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _pick_front_camera(fallback: str) -> str:
    """Resolve the front UVC by sysfs name; fall back to the given device.

    `/dev/video0` is NOT the front camera on this robot -- it is an Orbbec
    Gemini 335L depth stream. Opening it silently gives the segmenter a depth
    image and the mask looks plausible-but-wrong. The front UVC enumerates as
    "AX USB2.0" (and is the one advertising MJPG), measured 2026-11-19.
    """
    try:
        for path in sorted(glob.glob("/sys/class/video4linux/video*")):
            try:
                with open(os.path.join(path, "name"), encoding="utf-8") as handle:
                    name = handle.read().strip()
            except OSError:
                continue
            if "AX USB2.0" in name:
                return "/dev/" + os.path.basename(path)
    except Exception:
        pass
    return fallback


def _nodes(context, *args, **kwargs):
    role = LaunchConfiguration("role").perform(context).strip().lower()
    if role not in ("logic", "driver", "both"):
        raise RuntimeError(f"role 只接受 logic|driver|both，得到 {role!r}")

    allow_motion = LaunchConfiguration("allow_motion").perform(context).strip().lower()
    allow_motion = allow_motion in ("1", "true", "yes", "on")
    enabled = LaunchConfiguration("enabled").perform(context).strip().lower()
    enabled = enabled in ("1", "true", "yes", "on")
    controller = LaunchConfiguration("controller").perform(context).strip().lower()
    if controller not in ("direct", "nav2"):
        raise RuntimeError(f"controller 只接受 direct|nav2，得到 {controller!r}")

    image_topic = LaunchConfiguration("image_topic").perform(context)
    engine_path = LaunchConfiguration("engine_path").perform(context)
    rectify = LaunchConfiguration("rectify").perform(context).strip().lower()
    rectify = rectify in ("1", "true", "yes", "on")
    # With rectification on, the segmenter must eat the rectified stream (the
    # projection assumes undistorted pixels). An explicit image_topic still wins.
    if rectify and image_topic in ("", "/camera/image_raw"):
        image_topic = "/camera/image_rect"

    actions = []

    # ---------------- 驱动端：相机 + 矫正 + 分割（Jetson） ----------------
    if role in ("driver", "both"):
        camera_device = _pick_front_camera(
            LaunchConfiguration("camera_device").perform(context))
        camera_info_url = LaunchConfiguration("camera_info_url").perform(context).strip()
        node_list = [
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="camera",
                namespace="/camera",
                parameters=[dict([
                    ("video_device", camera_device),
                    ("pixel_format", "mjpeg2rgb"),
                    ("image_width", 640),
                    ("image_height", 480),
                    ("framerate", ParameterValue(
                        LaunchConfiguration("camera_fps"), value_type=float)),
                    ("camera_name", "front_camera"),
                    ("frame_id", "camera_optical_frame"),
                ] + ([("camera_info_url", camera_info_url)] if camera_info_url else []))],
                output="screen",
            ),
        ]
        if rectify:
            # lens distortion k1=0.24 / k2=-1.44 is not negligible: rectifying
            # moved the fitted camera height by 8% (0.347 -> 0.321 m) and cut
            # the projection residual from 15-22 px to 10-13 px (2026-11-19).
            # This is the stock rectifier, not our code.
            node_list.append(Node(
                package="image_proc",
                executable="rectify_node",
                name="rectify",
                namespace="/camera",
                remappings=[("image", "/camera/image_raw"),
                            ("camera_info", "/camera/camera_info")],
                parameters=[{"queue_size": 5}],
                output="screen",
            ))
        node_list.append(Node(
            package="rak_road_perception",
            executable="roadseg_node",
            name="roadseg_node",
            # Pass the RESOLVED topic, not LaunchConfiguration: the rectify
            # rewrite above happens in Python and would otherwise be ignored,
            # leaving the segmenter on the raw (distorted) stream.
            parameters=[dict([
                ("image_topic", image_topic),
                ("input_mode", "ros_image"),
                ("publish_nav_clouds", False),
            ] + ([("engine_path", engine_path)] if engine_path else []))],
            output="screen",
        ))
        actions += node_list

    # ---------------- 逻辑端：trace + 控制 + 预览（工作站） ----------------
    if role in ("logic", "both"):
        if controller == "nav2":
            # Stock Nav2 execution layer: road_goal_node publishes the traced
            # path, RotationShim + MPPI track it. Needs odom -> base_footprint.
            nav2_share = os.path.dirname(_CONFIG_DIR)
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_share, "launch", "nav2.launch.py")),
                launch_arguments={
                    "sensors": "camera",
                    "odom_source": LaunchConfiguration("odom_source"),
                    "motion_backend": LaunchConfiguration("motion_backend"),
                    "params_overlay": os.path.join(
                        _CONFIG_DIR, "vision_follow_params.yaml"),
                    "publish_robot_tf": LaunchConfiguration("publish_robot_tf"),
                    "slip_watch": "false",
                    "use_sim_time": "false",
                }.items(),
                condition=IfCondition(LaunchConfiguration("start_nav2")),
            ))
            actions.append(Node(
                package="rak_road_perception",
                executable="road_goal_node",
                name="road_goal_node",
                parameters=[dict([
                    _param("mask_topic", str),
                    _param("follow_action", str),
                    _param("odom_frame", str),
                    _param("base_frame", str),
                    _param("turn_policy", str),
                    ("enabled", bool(enabled)),
                    _param("lane_center_bias", float),
                ])],
                output="screen",
            ))
        else:
            follow_params = dict([
                _param("mask_topic", str),
                _param("cmd_topic", str),
                _param("max_vx", float),
                _param("max_vy", float),
                _param("max_wz", float),
                _param("turn_policy", str),
                _param("corner_follow", bool),
                _param("lookahead_frac", float),
                _param("k_lateral", float),
                _param("k_vy", float),
                _param("k_heading", float),
                _param("debug_image", bool),
                _param("debug_topic", str),
                ("enabled", bool(enabled)),
                ("control_hz", ParameterValue(
                    LaunchConfiguration("control_hz"), value_type=float)),
                _param("lane_center_bias", float),
            ])
            actions.append(Node(
                package="rak_road_perception",
                executable="road_follow_node",
                name="road_follow_node",
                parameters=[follow_params],
                output="screen",
            ))
            if enabled and allow_motion:
                # The ONLY writer of mecanum_drive_controller/reference. It
                # carries the measured roller-layout sign inversion; no other
                # node may publish to that topic (2026-11-19 unification).
                actions.append(Node(
                    package="rak_motion_stub",
                    executable="cmd_vel_omni",
                    name="cmd_vel_omni",
                    parameters=[dict([
                        _param("cmd_topic", str),
                        _param("reference_topic", str),
                        _param("reference_frame", str),
                    ])],
                    output="screen",
                ))
        if enabled and not allow_motion:
            actions.append(LogInfo(msg=(
                "road_follow: 解算中但 allow_motion:=false —— 只发命令话题，"
                "不写执行器。真机动车还需 chassis_bringup allow_motion:=true。"
            )))
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="road_follow_rviz",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="log",
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        _arg("role", "logic"),
        _arg("enabled", "false"),
        _arg("allow_motion", "false"),
        _arg("rviz", "true"),
        # 执行层二选一（唯一入口，不再有第二个 launch 文件）：
        #   direct —— road_follow_node 直接发 Twist/TwistStamped。不需要 TF、
        #             不背 Nav2 的 12+ 进程；跟踪质量是手写 P 控制的水平。
        #   nav2   —— road_goal_node 发 FollowPath，RotationShim + MPPI 跟踪。
        #             需要 odom -> base_footprint（EKF / 轮式里程计）。
        _arg("controller", "direct"),
        _arg("start_nav2", "true"),
        # nav2 档透传：odom_source 真机用 ekf，桌面夹具用 none（自己灌假里程计）；
        # motion_backend none 把 /cmd_vel 留给外层，sim 用 cmd_vel_sim 闭环。
        _arg("odom_source", "ekf"),
        _arg("motion_backend", "none"),
        _arg("follow_action", "/follow_path"),
        _arg("odom_frame", "odom"),
        _arg("base_frame", "base_link"),
        _arg("publish_robot_tf", "false"),
        _arg("rviz_config", _DEFAULT_RVIZ),
        _arg("mask_topic", "/road_perception/road_mask"),
        _arg("cmd_topic", "/cmd_vel"),
        _arg("reference_topic", "/mecanum_drive_controller/reference"),
        _arg("reference_frame", "base_link"),
        _arg("image_topic", "/camera/image_raw"),
        _arg("engine_path", ""),
        _arg("camera_device", "/dev/video0"),
        # Lens distortion k1=0.24 / k2=-1.44: rectifying changes the fitted
        # camera height by 8% and cuts the projection residual 15-22 px -> 10-13
        # px (measured 2026-11-19). GOAL_DEFAULTS is fitted on RECTIFIED frames,
        # so keep this on unless you refit.
        # Rectification is OFF by default ON PURPOSE. Wiring it needs a
        # projection matrix that PRESERVES the field of view
        # (cv2.getOptimalNewCameraMatrix), not P == K: with k1=0.24 / k2=-1.44
        # the naive P changed the bottom-row lane span from 636 to 478 px and
        # moved the fitted camera height from 0.347 to 0.52 m (measured
        # 2026-11-19). GOAL_DEFAULTS.cam_height_m / cam_pitch_rad are fitted on
        # RAW frames, so they pair with rectify:=false. Turn it on only after
        # generating a view-preserving camera_info and re-running
        # scripts/fit_camera_mount.py against rectified masks.
        _arg("rectify", "false"),
        _arg("camera_info_url", f"file://{_DEFAULT_CAMERA_INFO}"),
        _arg("camera_fps", "30.0"),
        _arg("control_hz", "20.0"),
        _arg("max_vx", "0.12"),
        _arg("max_vy", "0.08"),
        _arg("max_wz", "0.5"),
        _arg("turn_policy", "auto"),
        # 相机安装偏航 = 车道中心在图像里的恒定偏移。默认值来自实测（车停在车道
        # 正中，trace 稳定在 x_norm=+0.032，6 帧极差 0.0016）。换了相机/重装后
        # 必须重标：停正、读 trace 稳定段的 x，填这里。
        _arg("lane_center_bias", "0.032"),
        _arg("corner_follow", "true"),
        _arg("lookahead_frac", "0.70"),
        _arg("k_lateral", "0.7"),
        _arg("k_vy", "0.9"),
        _arg("k_heading", "0.9"),
        _arg("debug_image", "true"),
        _arg("debug_topic", "/road_perception/centerline_debug"),
        OpaqueFunction(function=_nodes),
    ])
