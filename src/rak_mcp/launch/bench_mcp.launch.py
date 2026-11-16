"""台架闭环：mock 底盘 + controller_manager + tool_executor。

MCP 服务本身**不**在这里起 —— 它必须独占 stdin/stdout 说 JSON-RPC，
所以由客户端（或人手）单独启动：

    ros2 run rak_mcp rak_mcp_server --allow-motion --name bench

本 launch 只负责让 `/mecanum_drive_controller/reference` 有人听、
`/joint_states` 与 odometry 有数据、`/tool_proposals` 有执行者。
控制器参数直接复用实机那份 `chassis_controllers.yaml`（不抄第二份轮几何）；
`tool_executor` 走 `rak_orchestration.launch_helpers.managed`，
与 `task.launch.py` 同一套生命周期写法（否则进程在、节点永远 unconfigured）。
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from rak_orchestration.launch_helpers import managed


def generate_launch_description() -> LaunchDescription:
    urdf_text = (Path(get_package_share_directory("rak_mcp")) / "bench"
                 / "mock_chassis.urdf").read_text(encoding="utf-8")
    controllers = (Path(get_package_share_directory("rak_bringup"))
                   / "config" / "chassis_controllers.yaml")
    robot_description = ParameterValue(urdf_text, value_type=str)

    # 这一版 controller_manager 从 /robot_description 话题取 URDF（不是参数），
    # 由 robot_state_publisher 以 transient_local 发布，晚起的节点也能拿到。
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[str(controllers)],
    )
    # 与 chassis_bringup.launch.py 同一写法：spawner 显式 --param-file 传控制器参数。
    # 只把文件交给 controller_manager 时，控制器节点读到的不是这一块，
    # 会以 "kinematics.wheels_radius ... floating point range" 失败（实测）。
    jsb_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30", "--param-file", str(controllers)],
    )
    mecanum_spawner = Node(
        package="controller_manager", executable="spawner", output="screen",
        arguments=["mecanum_drive_controller", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "30", "--param-file", str(controllers)],
    )

    return LaunchDescription([
        DeclareLaunchArgument("allow_motion", default_value="true"),
        DeclareLaunchArgument("audit_dir", default_value="data/episodes"),
        robot_state_publisher,
        control_node,
        jsb_spawner,
        mecanum_spawner,
        *managed(
            package="rak_tool_executor",
            executable="tool_executor",
            name="tool_executor",
            parameters=[{
                "allow_motion": LaunchConfiguration("allow_motion"),
                "audit_dir": LaunchConfiguration("audit_dir"),
            }],
            autostart=True,
        ),
    ])
