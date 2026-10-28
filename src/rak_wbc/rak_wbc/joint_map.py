"""[AI] 臂关节两套约定的单点映射（ADR-0011 接口边界）。

为什么存在：WBC 的运动学模型（Pinocchio）用 `rak_arm.urdf.xacro` 的 **q_urdf** 域，
而硬件/ros2_control 用**舵机指令角**域（`docs/arm_moveit_contract.md` §1，FROZEN）。
两域只差一个零位偏移（properties_arm.xacro：J2 零位 135°、J3 零位 180°，其余 0°）：

    q_urdf[i] = cmd_rad[i] - ARM_ZERO_Q[i]

历史上 WBC 节点直接拿 `/joint_states` 里的 `base_yaw_joint...` 名字去查（而
joint_state_broadcaster 发的是 `j1_base...`），既取不到状态也发错域。本模块把
"名字映射 + 域换算 + 饱和"集中定义，节点与测试共用。

不 import rclpy / numpy：纯 stdlib，桌面可直接单测。
"""
from __future__ import annotations

import math

# 运动学模型侧（rak_arm.urdf.xacro / rak_moveit_config）
ARM_URDF_JOINTS = (
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "forearm_roll_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
)
# 硬件控制侧（rak_chassis.urdf.xacro <ros2_control> / /joint_states）
ARM_CTRL_JOINTS = (
    "j1_base",
    "j2_shoulder",
    "j3_elbow",
    "j4_forearm",
    "j5_wrist_pitch",
    "j6_wrist_roll",
)
# 零位指令角（rad）= properties_arm.xacro 的 [J1 0°, J2 135°, J3 180°, J4 0°, J5 0°, J6 0°]
ARM_ZERO_Q = (0.0, 2.356194490192345, 3.141592653589793, 0.0, 0.0, 0.0)
# 指令角行程（rad）：0..270°，J2 硬件限 255°（4.45059）
ARM_CMD_MIN = 0.0
ARM_CMD_MAX = (
    4.71238898038469,
    4.45059,
    4.71238898038469,
    4.71238898038469,
    4.71238898038469,
    4.71238898038469,
)
# Home 安全位（q_urdf；= 指令 0/225/270/0/0/0）
HOME_Q = (0.0, 1.5707963267948966, 1.5707963267948966, 0.0, 0.0, 0.0)

_N = len(ARM_URDF_JOINTS)


def _as_floats(values, what):
    out = [float(v) for v in values]
    if len(out) != _N:
        raise ValueError(f"{what} 长度 {len(out)} != {_N}")
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{what} 含非有限值")
    return out


def ctrl_to_q(ctrl_values, zero=None):
    """指令角域（硬件/ros2_control）-> q_urdf（运动学模型）。

    zero 省略时用 ARM_ZERO_Q；传入运行期标定值（节点参数 arm_zero_q）可覆盖。
    """
    values = _as_floats(ctrl_values, "ctrl_values")
    offsets = ARM_ZERO_Q if zero is None else _as_floats(zero, "arm_zero_q")
    return [value - offset for value, offset in zip(values, offsets)]


def q_to_ctrl(q_values, zero=None):
    """q_urdf（运动学模型）-> 指令角域（硬件/ros2_control）。"""
    values = _as_floats(q_values, "q_values")
    offsets = ARM_ZERO_Q if zero is None else _as_floats(zero, "arm_zero_q")
    return [value + offset for value, offset in zip(values, offsets)]


def clamp_cmd(ctrl_values):
    """把指令角域值饱和到物理行程内（J2 上限 255°）。

    与固件 Session 层饱和同口径（负值截 0，越上界截上界），也等同
    controller_manager 的 JointSaturationLimiter（URDF command_interface min/max）。
    """
    values = _as_floats(ctrl_values, "ctrl_values")
    return [min(max(value, ARM_CMD_MIN), hi) for value, hi in zip(values, ARM_CMD_MAX)]
