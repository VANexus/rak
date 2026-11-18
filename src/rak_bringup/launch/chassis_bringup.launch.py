"""[AI] 底盘 + 臂 CANopen v5 控制链 bringup（ADR-0024：主站 = ros2_canopen）。

启动 robot_state_publisher + ros2_canopen 主站（device_container，NMT/SYNC/PDO
归它）+ controller_manager（硬件插件 = `rak_chassis/CanopenChassis|CanopenArm`
薄桥，经 proxy driver 的 rpdo/tpdo 话题交换 OD 对象），然后 spawn
joint_state_broadcaster、mecanum_drive_controller、arm_controller(JTC) 与
gripper_controller。上层工具只经 ROS 接口对话，不碰 CAN（ADR-0016 决策 2 保留）。

臂运动写者二选一（arm_motion_writer，ADR-0011 D1）：jtc（默认）/ wbc / none，
语义与旧 v4 文件一致；真机 MoveIt 档依赖 jtc。

安全:
- allow_motion 默认 false —— 桥只发零目标，cmd_flags.bit0=0，臂命令一帧不发。
- allow_motion:=true 是显式运动许可，前提: 车轮架空 + 电池可控 + 有急停手段。

台架（无下位机）验证：can_interface:=vcan0 + canopen_fake_slaves（见
docs/status/canopen_vcan_2026-11-14.md）。真机：can_interface:=can0（默认）。

用法:
  ros2 launch rak_bringup chassis_bringup.launch.py
  ros2 launch rak_bringup chassis_bringup.launch.py allow_motion:=true
  ros2 launch rak_bringup chassis_bringup.launch.py arm_motion_writer:=wbc
  # 主站跑在物理连 can0 的另一台机器（本机无 CAN 口）：
  ros2 launch rak_bringup chassis_bringup.launch.py canopen_container:=external
"""

import tempfile
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue



def _launch_setup(context, *args, **kwargs):
    def config(name):
        return LaunchConfiguration(name).perform(context)

    allow_motion = config("allow_motion")
    arm_writer = config("arm_motion_writer")
    can_interface = config("can_interface")
    controllers_file = config("controllers_file")
    master_config = config("master_config")
    urdf_path = config("urdf")
    slaves = config("slaves")
    container_mode = config("canopen_container")
    if container_mode not in ("spawn", "external"):
        raise RuntimeError(
            f"canopen_container 必须是 spawn|external，得到 {container_mode!r}")
    if slaves not in ("both", "chassis", "arm"):
        raise RuntimeError(f"slaves 必须是 both|chassis|arm，得到 {slaves!r}")

    stabilizer = config("stabilizer").split()[0].lower()
    assert stabilizer in ("true", "false"), f"stabilizer 只接受 true/false，得到 {stabilizer!r}"
    sign = config("wheel_sign").split()
    assert len(sign) == 4, f"wheel_sign 需 4 个值（FL FR RL RR），得到 {sign!r}"
    sign_fl, sign_fr, sign_rl, sign_rr = sign

    robot_description = ParameterValue(
        Command(
            f"xacro {urdf_path} allow_motion:={allow_motion} "
            f'wheel_sign_fl:="{sign_fl}" wheel_sign_fr:="{sign_fr}" '
            f'wheel_sign_rl:="{sign_rl}" wheel_sign_rr:="{sign_rr}" '
            f"slaves:={slaves} stabilizer:={stabilizer}"
        ),
        value_type=str,
    )

    canopen_share = get_package_share_directory("rak_canopen")

    # ---- 1) ros2_canopen 主站容器（NMT master node 127 + proxy driver ×2）----
    # 2026-11-14 实测（vcan + fake slave，docs/status/canopen_vcan_2026-11-14.md）：
    #  - device_container **不是** lifecycle 节点，configure/activate 无用；
    #  - master_config 必须是 dcfgen 生成的主站 DCF（带 1F80/1F81/1F85 网络表），
    #    传从站 EDS 会 Boot Timeout 后 abort；
    #  - 从站 dcf 必须是相对名，dcf_path 是它们的父目录 → bus.yaml 里的
    #    @BUS_CONFIG_PATH@ 在这里替换成安装态 config 目录。
    # canopen_container:=external 时**不在本机起**这个容器：主站必须待在物理连着
    # can0 的那台（orin），本机只跑 controller_manager 与控制器；两侧经
    # ros2_canopen proxy driver 的 `<driver>/rpdo|tpdo` 话题跨 DDS 对接，不新增协议层。
    device_container = None
    if container_mode == "spawn":
        canopen_config = canopen_share + "/config"
        bus_text = Path(canopen_config + "/bus.yaml").read_text(encoding="utf-8")
        bus_text = bus_text.replace("@BUS_CONFIG_PATH@", canopen_config)
        # 从站裁剪（slaves:=chassis|arm|both）：单板联调时把另一台从站从 bus.yaml 摘掉。
        # 不做裁剪就必须两台都在线 —— 缺席的那台会 Boot Timeout，容器直接 abort（上游行为）。
        if slaves != "both":
            sections, current, key = [], [], None
            for line in bus_text.splitlines(keepends=True):
                if line and not line[0].isspace() and not line.startswith("#"):
                    if current:
                        sections.append((key, current))
                    current, key = [line], line.split(":")[0].strip()
                else:
                    current.append(line)
            if current:
                sections.append((key, current))
            keep_keys = {"options", "master", slaves}
            bus_text = "".join("".join(lines) for k, lines in sections if k in keep_keys)
        bus_file = tempfile.NamedTemporaryFile(
            mode="w", prefix="rak_canopen_bus_", suffix=".yaml", delete=False)
        bus_file.write(bus_text)
        bus_file.close()

        container_params = {
            "bus_config": bus_file.name,
            "can_interface_name": can_interface,
            "master_config": master_config,
        }
        device_container = Node(
            package="canopen_core",
            executable="device_container_node",
            name="device_container",
            namespace="",
            output="screen",
            parameters=[container_params],
        )

    device_nodes = [device_container] if device_container is not None else []

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        *device_nodes,
        # bus.yaml 用**非 lifecycle** 驱动（MasterDriver / ProxyDriver）：容器在
        # init() 里自己 configure+activate 主站与各驱动，没有 `/lifecycle_manager`
        # 节点，也就不再需要 6 s/9 s 定时敲 `ros2 lifecycle set`（2026-11-15 移除：
        # 定时转移是启动时序赌博，非 lifecycle 路径已在 vcan0 上端到端验证）。
        # 等驱动的 NMT reset → boot-up → 激活跑完再起 controller_manager；硬件插件
        # 自己还有 startup_grace_ms=5000 的宽限，两层都不靠"刚好赶上"。
        TimerAction(period=6.0, actions=[Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            output="screen",
            parameters=[
                controllers_file,
                {"robot_description": robot_description},
            ],
        )]),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_file],
            output="screen",
        ),
    ]
    # 底盘控制器只在底盘段存在时 spawn（URDF 里没那段就没有对应命令接口）。
    if slaves in ("both", "chassis"):
        nodes.append(Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mecanum_drive_controller",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controllers_file],
            output="screen",
        ))

    # [AI] 校验与 spawn 分离：slaves:=chassis 时 arm 写者不参与（没有臂段就
    # 没有 position 接口），但值仍必须合法。2026-11-16 修复：旧 elif 链在
    # chassis-only + jtc 时会把"该档不适用"误判成非法值而直接拒绝启动。
    if arm_writer not in ("jtc", "wbc", "none"):
        raise RuntimeError(f"arm_motion_writer 必须是 jtc|wbc|none，得到 {arm_writer!r}")
    if arm_writer == "jtc" and slaves in ("both", "arm"):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_controller",
                           "--controller-manager", "/controller_manager",
                           "--param-file", controllers_file],
                output="screen",
            )
        )
    elif arm_writer == "wbc" and slaves in ("both", "arm"):
        nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["arm_position_controller",
                           "--controller-manager", "/controller_manager",
                           "--param-file", controllers_file,
                           "--controller-ros-args",
                           "-r", "/arm_position_controller/~/commands:=/wbc/arm_setpoints"],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    description_share = get_package_share_directory("rak_description")
    bringup_share = get_package_share_directory("rak_bringup")

    return LaunchDescription([
        DeclareLaunchArgument(
            "allow_motion", default_value="false",
            description="运动许可；false=硬件只收遥测，命令恒 0"),
        DeclareLaunchArgument("arm_motion_writer", default_value="jtc",
                              description="臂控制器二选一：jtc / wbc / none"),
        DeclareLaunchArgument("can_interface", default_value="can0",
                              description="SocketCAN 接口；台架用 vcan0"),
        DeclareLaunchArgument(
            "canopen_container", default_value="spawn",
            description="spawn=本机起 device_container（需本机有 CAN 口）；"
                        "external=主站已在物理连 can0 的另一台机器上跑"
                        "（跨机经 <driver>/rpdo|tpdo 话题对接）"),
        DeclareLaunchArgument(
            "master_config",
            default_value=get_package_share_directory("rak_canopen") + "/config/master.dcf",
            description="ros2_canopen 主站 DCF（官方 dcfgen 从 bus.yaml 生成，见 rak_canopen）"),
        DeclareLaunchArgument(
            "slaves", default_value="both",
            description="从站裁剪：both|chassis|arm（单板联调时摘掉缺席的那台，"
                        "否则它会 Boot Timeout 让容器 abort）"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=bringup_share + "/config/chassis_controllers.yaml",
            description="控制器参数文件（CANopen 链路唯一一份）"),
        DeclareLaunchArgument(
            "urdf",
            default_value=description_share + "/urdf/rak_chassis.urdf.xacro"),
        DeclareLaunchArgument(
            # 物理方向权威在固件 wheel_config.h；2026-11-16/17 实测命令侧全 +1 时
            # 车直行，Jetson 不再叠加第二份符号。反馈极性是每关节 state_sign 的事
            # （固件已乘 encoder_sign，主机保持 +1），不要在这里改。注意负数会被
            # ros2 launch 当选项，CLI 覆盖必须写成 wheel_sign:="1 1 1 1" 带引号形式。
            "wheel_sign", default_value="1 1 1 1",
            description="四轮极性 FL FR RL RR；+1 正常，-1 命令与反馈同时取反"),
        DeclareLaunchArgument(
            # 固件 R74 的 cmd_flags.bit1：盘上 IMU 的 yaw rate 闭环修四轮目标。
            # 这是"直走很歪"的第一手段；默认关，A/B 用 true 打开。
            "stabilizer", default_value="false",
            description="轮域陀螺稳定器（固件内环）：true 打开，直走抗偏"),
        OpaqueFunction(function=_launch_setup),
    ])
