"""[AI] L2 组件化: 感知容器（composable component container）。

把感知链的 C++ 节点装进一个进程容器, 同容器内开 use_intra_process_comms
(进程内零拷贝: 免除序列化与网络栈)。收益量化见 docs/perf/ipc_baseline.md §2/§4:
跨进程 61MB/s p50 ~2.3-3.7ms / 92MB/s 曾丢至 60%(负载敏感) → 进程内零拷贝
p50 0.14-0.18ms / 零丢帧 / CPU 1-3%。

可加载组件 (2026-10-14 起):
  · gpu_pointcloud  —— 本包, 默认加载 (depth -> points, 容器的常驻云节点);
  · astra           —— camera:=astra 时加载 (Astra S 深度/彩色流, 组件化于本日);
  · stereo          —— stereo:=true 时加载 (双目 SBS 采集, 源早已是注册组件)。
并入记录与配方(步骤固定, 见下方四步):
  · rak_astra_driver 已于 2026-10-14 完成同一四步组件化并合入本容器入口;
  · 若再并入其它节点, 四步 = 与 astra 相同:
      (1) 源: 构造改为 (const rclcpp::NodeOptions & = rclcpp::NodeOptions()),
              #include <rclcpp_components/register_node_macro.hpp>,
              类外 RCLCPP_COMPONENTS_REGISTER_NODE(<ns>::<Class>);
      (2) CMake: find_package(rclcpp_components) + add_library(<x>_component SHARED <src>)
              + rclcpp_components_register_node(<x>_component EXECUTABLE <原可执行名>);
      (3) package.xml: <depend>rclcpp_components</depend>;
      (4) 本 launch 加 ComposableNode(package="...", plugin=<注册类名>, ...)，
          且 rak_pointcloud/package.xml 需补对应 exec_depend（门禁要求, 已补 astra/stereo）。

用法:
  ros2 launch rak_pointcloud perception_container.launch.py                 # 仅点云, intra 默认开
  ros2 launch rak_pointcloud perception_container.launch.py camera:=astra   # + 深度相机(容器内零拷贝链)
  ros2 launch rak_pointcloud perception_container.launch.py stereo:=true    # + 双目采集
  ros2 launch rak_pointcloud perception_container.launch.py intra:=false    # 对照(仍走 DDS)
  ros2 component list                                                       # 容器内组件清单

注意: bringup (sensors/depth_camera/grasp_camera) 仍按原样起独立进程, 行为不变;
本容器是"要零拷贝时"的显式入口, 尚未接为默认。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def _pc_node(intra_enabled: bool, condition) -> ComposableNode:
    return ComposableNode(
        package="rak_pointcloud",
        plugin="rak::pointcloud::GpuPointCloudNode",
        name="gpu_pointcloud",
        # 进程内零拷贝开关: 真 bool（不能用 LaunchConfiguration 直传, 参数类型会变字符串）
        extra_arguments=[{"use_intra_process_comms": intra_enabled}],
        parameters=[{
            "depth_topic": LaunchConfiguration("depth_topic"),
            "camera_info_topic": LaunchConfiguration("info_topic"),
            "points_topic": LaunchConfiguration("points_topic"),
            "decimation": ParameterValue(LaunchConfiguration("decimation"), value_type=int),
        }],
        condition=condition,
    )


def _astra_node(intra_enabled: bool, condition) -> ComposableNode:
    return ComposableNode(
        package="rak_astra_driver",
        plugin="AstraBridgeNode",
        name="astra_bridge",
        extra_arguments=[{"use_intra_process_comms": intra_enabled}],
        parameters=[{
            "color_frame_id": LaunchConfiguration("color_frame_id"),
            "depth_frame_id": LaunchConfiguration("depth_frame_id"),
        }],
        condition=condition,
    )


def _stereo_node(intra_enabled: bool, condition) -> ComposableNode:
    return ComposableNode(
        package="rak_stereo_driver",
        plugin="rak::stereo_driver::CaptureNode",
        name="eye_capture",
        extra_arguments=[{"use_intra_process_comms": intra_enabled}],
        parameters=[{
            "device": LaunchConfiguration("device"),
        }],
        condition=condition,
    )


def generate_launch_description():
    intra = LaunchConfiguration("intra")
    camera = LaunchConfiguration("camera")
    stereo = LaunchConfiguration("stereo")

    on, off = IfCondition(intra), UnlessCondition(intra)

    def _pair(prefix):
        """组件条件与 intra 开关求与: (intra on 条件, intra off 条件)。

        两者必须互斥且合起来覆盖该组件的加载条件——只用组件条件做 If/Unless 会把
        intra 开关弄丢(intra:=false 时组件仍以 intra=true 加载)。
        """
        on_expr = PythonExpression(prefix + [" and '", intra, "' == 'true'"])
        off_expr = PythonExpression(prefix + [" and '", intra, "' != 'true'"])
        return IfCondition(on_expr), IfCondition(off_expr)

    astra_cond = _pair(["'", camera, "' == 'astra'"])
    stereo_cond = _pair(["'", stereo, "' == 'true'"])

    container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            _pc_node(True, on),
            _pc_node(False, off),
            _astra_node(True, astra_cond[0]),
            _astra_node(False, astra_cond[1]),
            _stereo_node(True, stereo_cond[0]),
            _stereo_node(False, stereo_cond[1]),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("intra", default_value="true",
                              description="同容器组件走进程内零拷贝(免序列化/免网络栈)"),
        DeclareLaunchArgument("camera", default_value="none",
                              description="随容器加载的深度相机组件: none|astra"),
        DeclareLaunchArgument("stereo", default_value="false",
                              description="true=随容器加载双目采集组件"),
        DeclareLaunchArgument("color_frame_id", default_value="camera_color_frame"),
        DeclareLaunchArgument("depth_frame_id", default_value="camera_depth_frame"),
        DeclareLaunchArgument("device", default_value="/dev/video0"),
        DeclareLaunchArgument("decimation", default_value="1",
                              description="点云抽稀系数(1=全分辨率)"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("points_topic", default_value="/camera/depth/points_gpu"),
        container,
    ])