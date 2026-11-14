"""真实六轴臂的 MoveIt 启动 —— 两种模式：本地仿真 / 连真机。

装配的是仓库里已有的文件，不新增模型：
  * 几何   rak_description/urdf/rak_arm.urdf.xacro（真实几何，非夹具；
           其运动链来自 macros/arm_chain.xacro，与整车 URDF 共用同一个宏）
  * 语义   config/rak_arm.srdf（规划组 arm：arm_base_link -> tool0）
  * IK     config/rak_arm_kinematics.yaml（KDL，MoveIt 自带）
  * 规划   OMPL
  * 执行   config/rak_arm_controllers.yaml → controller_names: [arm_controller]

两种模式：

  use_hardware:=false（默认，**安全默认**）
      本地仿真：起 controller_manager(mock_components/GenericSystem) + 两个 spawner
      + robot_state_publisher + move_group + 可选 rviz2。全程不碰任何设备。

  use_hardware:=true
      **连真机上已经在跑的那套**：不再起 controller_manager / spawner /
      robot_state_publisher —— 它们由 `rak_bringup chassis_bringup.launch.py` 起了
      （硬件插件 rak_chassis/CanopenArm → ros2_canopen 主站 → CANopen v5 → F407）。
      MoveIt 只起 move_group（+ 可选 rviz2），靠同名的 `arm_controller`
      （joint_trajectory_controller）对接 —— 关节名与 action 名在两侧逐字一致。
      臂的 TF 由**真机的** robot_state_publisher 发布
      （整车 URDF 里已有真实臂几何），所以本机不需要第二个 rsp：
      两个 rsp 会同时往 /robot_description 发，消费者拿到哪个是随机的。

⚠️ 真机档的前置条件（不在本 launch 里做，属于操作者的责任）：
  1. 本机要能发现真机：RMW 必须是 rmw_cyclonedds_cpp、ROS_DOMAIN_ID=47、
     CYCLONEDDS_URI 指向 rak_bringup/config/cyclonedds.xml
     （见 src/rak_bringup/scripts/environment.sh:28-30）。⚠️ 用户 ~/.bashrc 里那条
     CYCLONEDDS_URI 语法是坏的，必须覆盖它。
  2. 真机那边 arm_controller 已 spawn（chassis_bringup.launch.py）。
  3. 真要它动：chassis_bringup 必须 allow_motion:=true。
     ⚠️ 那个开关**同时解禁轮子**，台架上要两个都防住。
  4. 舵机无编码器 —— /joint_states 是固件指令的回声，不是实测位置。
     MoveIt 发完轨迹就认为到位，**它不知道真机有没有真的到**（契约 §2.2）。

用法：
    ros2 launch rak_moveit_config rak_arm.launch.py                      # 本地仿真 + RViz
    ros2 launch rak_moveit_config rak_arm.launch.py rviz:=false          # 仿真，headless
    ros2 launch rak_moveit_config rak_arm.launch.py use_hardware:=true   # 连真机（域 47）
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_share = get_package_share_directory("rak_moveit_config")
    description_share = get_package_share_directory("rak_description")

    urdf_path = os.path.join(description_share, "urdf", "rak_arm.urdf.xacro")
    srdf_path = os.path.join(moveit_share, "config", "rak_arm.srdf")
    kinematics_path = os.path.join(moveit_share, "config", "rak_arm_kinematics.yaml")
    controllers_path = os.path.join(moveit_share, "config", "rak_arm_controllers.yaml")
    ros2_control_path = os.path.join(moveit_share, "config", "rak_arm_ros2_control.yaml")
    limits_path = os.path.join(moveit_share, "config", "rak_arm_joint_limits.yaml")
    rviz_config = os.path.join(moveit_share, "config", "rak_arm_moveit.rviz")

    # [AI] 真机档 = 真机那边已经把 controller_manager / spawner / rsp 都起了，
    # 本 launch 再起一份会撞：两个 /controller_manager、两个 /robot_description、
    # 两个 arm_controller(JTC) 和两个 joint_state_broadcaster
    # （/joint_states 会在真假值之间跳）。
    sim_only = UnlessCondition(LaunchConfiguration("use_hardware"))
    hw_only = IfCondition(LaunchConfiguration("use_hardware"))

    moveit_config = (
        MoveItConfigsBuilder("rak", package_name="rak_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path=srdf_path)
        .robot_description_kinematics(file_path=kinematics_path)
        # [AI] 必须给：URDF 的 <limit> 没有加速度字段，缺 max_acceleration 时
        # MoveIt 的 AddTimeOptimalParameterization 会直接 FAILURE（Plan 必失败）。
        .joint_limits(file_path=limits_path)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        # [AI] 必须显式给：包里有 fake_controllers.yaml / six_axis_sim_controllers.yaml
        # 两个候选，builder 的自动搜索会因"猜不出用哪个"而抛错。
        .trajectory_execution(file_path=controllers_path)
        .to_moveit_configs()
    )

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        # 真机档不起：真机的 rsp 发的是 rak_chassis.urdf.xacro，那份里已经有
        # 真实的臂几何（2026-10-15 并入），所以本机不需要第二个 rsp。
        condition=sim_only,
    )

    # [AI] ros2_control 仿真链路：controller_manager + 两个 spawner。
    # 硬件插件是 mock_components/GenericSystem（写在 rak_arm.urdf.xacro 里），
    # 纯内存模拟 —— 这条链碰不到 CAN / 串口 / 任何真实设备。
    # joint_state_broadcaster 负责发 /joint_states（MoveIt 的当前状态靠它），
    # arm_controller(JTC) 负责接 MoveIt 发来的轨迹 → Execute 才有落点。
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[ros2_control_path, {"robot_description": robot_description}],
        condition=sim_only,
    )

    spawner_jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager",
                   "--param-file", ros2_control_path],
        output="screen",
        condition=sim_only,
    )

    spawner_arm = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller",
                   "--controller-manager", "/controller_manager",
                   "--param-file", ros2_control_path],
        output="screen",
        condition=sim_only,
    )

    # [AI] move_group 建两份，条件互补 —— 因为"什么时候该起"在两种模式下不一样：
    #   仿真档：必须等 arm_controller 起来。MoveItSimpleControllerManager 在加载
    #           控制器句柄时就创建 action client，此刻 JTC 若不存在，Execute 会一直报
    #           "Action client not connected to action server"（本机 JTC 要 20~30 秒
    #           才 loaded+activated）。所以由 OnProcessExit(spawner_arm) 触发。
    #   真机档：spawner 被禁用 → 那个事件**永不触发** → 必须无条件直接起。
    #           （这是加开关时最容易踩的坑：把 move_group 留在 on_exit 里，真机档
    #            会静默地什么都不起。）
    def make_move_group(condition=None):
        return Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[moveit_config.to_dict()],
            condition=condition,
        )

    move_group_sim = make_move_group()      # 由 OnProcessExit 启动，不进 LaunchDescription
    move_group_hw = make_move_group(hw_only)

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="log",
        condition=IfCondition(LaunchConfiguration("rviz")),
        # [AI] RViz 的 MotionPlanning 插件在**自己的节点**上找 robot_description_semantic /
        # robot_description_kinematics —— 只喂给 move_group 不够，会报
        # "Could not find parameter robot_description_semantic"。这里按官方 demo 的口径
        # 把模型/语义/运动学/规划管线/限位一并给它。
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="是否启动 RViz（headless 时设 false）"),
        DeclareLaunchArgument(
            "use_hardware", default_value="false",
            description="true=连真机上已由 chassis_bringup 起的 arm_controller"
                        "（不起本地 controller_manager / spawner / rsp）；"
                        "false=本地 mock 仿真"),
        rsp,
        controller_manager,
        spawner_jsb,
        # [AI] 仿真档：move_group 等 arm_controller 起来之后再启动（理由见
        # make_move_group 上方的注释）。真机档下 spawner_arm 被条件禁用，
        # 本事件不会触发，改由下面的 move_group_hw 直接起。
        RegisterEventHandler(
            OnProcessExit(target_action=spawner_arm, on_exit=[move_group_sim])
        ),
        spawner_arm,
        # 真机档：不等任何人，直接起（真机的 arm_controller 由操作者保证已在跑）。
        move_group_hw,
        rviz,
    ])
