"""[AI] L2 组件化: 感知容器（composable component container）。

把感知链的 C++ 节点装进一个进程容器, 同容器内开 use_intra_process_comms
(进程内零拷贝: 免除序列化与网络栈)。收益量化见 docs/perf/ipc_baseline.md §4:
跨进程 61MB/s p50 ~3.6ms / 92MB/s 丢 40% → 进程内零拷贝 p50 0.14-0.18ms / 零丢帧 / CPU 1-3%。

现状: 已组件化并在本容器可加载的只有 gpu_pointcloud (本包)。
后续并入(待其改动落定, 步骤固定):
  · rak_astra_driver/astra_bridge_node.cpp —— 该文件当前有其他会话在制品, 勿并行改;
    落定后的组件化四步 = 与本次 gpu_pointcloud 相同:
      (1) 源: 构造改为 (const rclcpp::NodeOptions & = rclcpp::NodeOptions()),
              #include <rclcpp_components/register_node_macro.hpp>,
              类外 RCLCPP_COMPONENTS_REGISTER_NODE(<ns>::<Class>);
      (2) CMake: find_package(rclcpp_components) + add_library(<x>_component SHARED <src>)
              + rclcpp_components_register_node(<x>_component EXECUTABLE <原可执行名> COMPONENT <name>);
      (3) package.xml: <depend>rclcpp_components</depend>;
      (4) 本 launch 加 ComposableNode(package="rak_astra_driver", plugin=<注册类名>, ...)。
    届时 rak_pointcloud/package.xml 需加 exec_depend rak_astra_driver（门禁要求）。
  · rak_stereo_driver/capture —— 源已是注册组件, 直接加 ComposableNode 即可。

用法:
  ros2 launch rak_pointcloud perception_container.launch.py                 # intra 默认开
  ros2 launch rak_pointcloud perception_container.launch.py intra:=false    # 对照(仍走 DDS)
  ros2 component list                                                       # 容器内组件清单
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
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


def generate_launch_description():
    intra = LaunchConfiguration("intra")

    container = ComposableNodeContainer(
        name="perception_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            _pc_node(True, IfCondition(intra)),
            _pc_node(False, UnlessCondition(intra)),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("intra", default_value="true",
                              description="同容器组件走进程内零拷贝(免序列化/免网络栈)"),
        DeclareLaunchArgument("decimation", default_value="1",
                              description="点云抽稀系数(1=全分辨率)"),
        DeclareLaunchArgument("depth_topic", default_value="/camera/depth/image_raw"),
        DeclareLaunchArgument("info_topic", default_value="/camera/depth/camera_info"),
        DeclareLaunchArgument("points_topic", default_value="/camera/depth/points_gpu"),
        container,
    ])