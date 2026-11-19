"""纯视觉寻路 —— 走上现成 Nav2 控制层（2026-11-19 用户决策）。

数据流（全部标准话题 / 标准 action）：

    roadseg_node  --/road_perception/road_mask-->  road_goal_node
                                                       |  FollowPath (nav2_msgs)
                                                       v
                                    controller_server: RotationShim -> MPPI(Omni)
                                                       |  /cmd_vel
                                                       v
                                        (外层底盘适配器 / TWIST -> TWISTSTAMPED)

为什么这样接（不要自己写控制律）：
  * 「转弯转不过去」= 路径朝向与车头差得远，却没有原地转这一步
    -> `nav2_rotation_shim_controller`（已装，`config/vision_follow_params.yaml`）。
  * 「走不到路中间」= 手写 P 控制的稳态误差
    -> `nav2_mppi_controller` 是轨迹优化器，这个问题在它那里不成立。
  * 「直角弯分不清左右」**没有现成件**：它在 `trace_centerline` 里解（那是本仓
    唯一保留的视觉判别逻辑），输出就是一条真的会拐弯的折线。

运动许可：本文件本身不动底盘。`enabled:=true` 只让 vision 链路出 /cmd_vel；
真机动车仍要 `chassis_bringup.launch.py allow_motion:=true` 且 Nav2 是唯一的
/cmd_vel 写者（ADR-0011）。

    # 驱动端（Jetson）
    ros2 launch rak_bringup road_follow_nav2.launch.py role:=driver

    # 逻辑端（工作站）
    ros2 launch rak_bringup road_follow_nav2.launch.py role:=logic enabled:=true

前提：`odom -> base_footprint` 必须有人发布（EKF / rf2o / 轮式），
`road_goal_node` 靠它把机体坐标系路径投到 odom。仅预览可以
`publish_robot_tf:=true`，但那只是机体树，不是里程计。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

try:
    from ament_index_python.packages import get_package_share_directory

    _SHARE = get_package_share_directory("rak_bringup")
except Exception:  # source tree without a local install
    _SHARE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def _arg(name, default):
    return DeclareLaunchArgument(name, default_value=default)


def _param(name, value_type):
    return name, ParameterValue(LaunchConfiguration(name), value_type=value_type)


def _nodes(context, *args, **kwargs):
    role = LaunchConfiguration("role").perform(context).strip().lower()
    if role not in ("logic", "driver", "both"):
        raise RuntimeError(f"role 只接受 logic|driver|both，得到 {role!r}")

    enabled = LaunchConfiguration("enabled").perform(context).strip().lower()
    enabled = enabled in ("1", "true", "yes", "on")

    actions = []

    if role in ("driver", "both"):
        driver = os.path.join(_SHARE, "launch", "road_follow.launch.py")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver),
            launch_arguments={
                "role": "driver",
                "camera_device": LaunchConfiguration("camera_device"),
                "camera_fps": LaunchConfiguration("camera_fps"),
                "image_topic": LaunchConfiguration("image_topic"),
                "engine_path": LaunchConfiguration("engine_path"),
                "rviz": "false",
            }.items(),
        ))

    if role in ("logic", "both"):
        overlay = os.path.join(_SHARE, "config", "vision_follow_params.yaml")
        nav2 = os.path.join(_SHARE, "launch", "nav2.launch.py")
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2),
            launch_arguments={
                "sensors": "camera",          # 纯视觉：没有雷达，不跑 slam_toolbox
                "odom_source": "ekf",         # camera 档 rf2o 无 /scan，里程计只能来自 EKF
                "motion_backend": "none",     # /cmd_vel 留给外层底盘适配器
                "params_overlay": overlay,    # RotationShim + MPPI
                "publish_robot_tf": LaunchConfiguration("publish_robot_tf"),
                "slip_watch": LaunchConfiguration("slip_watch"),
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
                _param("path_topic", str),
                _param("follow_action", str),
                _param("odom_frame", str),
                _param("base_frame", str),
                _param("resend_period_s", float),
                _param("path_hz", float),
                _param("cam_height_m", float),
                _param("cam_pitch_rad", float),
                _param("path_len_m", float),
                _param("turn_policy", str),
                ("enabled", bool(enabled)),
            ])],
            output="screen",
        ))
        actions.append(Node(
            package="rviz2",
            executable="rviz2",
            name="road_follow_rviz",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(LaunchConfiguration("rviz")),
            output="log",
        ))

    return actions


def generate_launch_description():
    default_rviz = os.path.join(_SHARE, "config", "road_follow.rviz")
    return LaunchDescription([
        _arg("role", "logic"),
        _arg("enabled", "false"),
        _arg("start_nav2", "true"),
        _arg("rviz", "true"),
        _arg("rviz_config", default_rviz),
        _arg("mask_topic", "/road_perception/road_mask"),
        _arg("path_topic", "/vision_path"),
        _arg("follow_action", "/follow_path"),
        _arg("odom_frame", "odom"),
        _arg("base_frame", "base_link"),
        _arg("resend_period_s", "2.0"),
        _arg("path_hz", "5.0"),
        # [AI] 占位：相机高度/俯仰未实测，只影响米制投影的尺度（见
        # docs/design/vision_nav_v1.md §5A）。量过再回填，否则速度会成比例偏差。
        _arg("cam_height_m", "0.30"),
        _arg("cam_pitch_rad", "0.0"),
        _arg("path_len_m", "1.8"),
        _arg("turn_policy", "auto"),
        _arg("camera_device", "/dev/video0"),
        _arg("camera_fps", "30.0"),
        _arg("image_topic", "/camera/image_raw"),
        _arg("engine_path", ""),
        _arg("publish_robot_tf", "false"),
        _arg("slip_watch", "false"),
        OpaqueFunction(function=_nodes),
    ])
