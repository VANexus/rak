"""joint_map 的域换算回归（纯 stdlib；不依赖 rclpy / pinocchio / numpy）。

[AI] 为什么存在：WBC 的臂状态读取曾用模型侧名字去查 /joint_states（永远匹配不到），
且发布值落在 q_urdf 域而硬件期待指令角域——都是"不报错的静默错"，只能靠断言挡住。
"""
import math

import pytest

from rak_wbc import joint_map

DEG = 180.0 / 3.141592653589793


def test_joint_lists_are_parallel_and_distinct():
    assert len(joint_map.ARM_URDF_JOINTS) == len(joint_map.ARM_CTRL_JOINTS) == 6
    assert not set(joint_map.ARM_URDF_JOINTS) & set(joint_map.ARM_CTRL_JOINTS)


def test_zero_q_matches_properties_arm_zero_positions():
    # properties_arm.xacro：J2 零位 135°、J3 零位 180°，其余 0°
    deg = [value * DEG for value in joint_map.ARM_ZERO_Q]
    assert [round(v, 6) for v in deg] == [0.0, 135.0, 180.0, 0.0, 0.0, 0.0]


def test_home_q_maps_to_firmware_home_command():
    # Home 指令 = J1..J6 0/225/270/0/0/0（R44 上电默认位）
    ctrl = joint_map.q_to_ctrl(joint_map.HOME_Q)
    assert [round(v * DEG, 4) for v in ctrl] == [0.0, 225.0, 270.0, 0.0, 0.0, 0.0]


def test_round_trip_is_identity():
    ctrl = [0.3, 0.9, 1.2, 2.5, 0.0, 4.0]
    back = joint_map.ctrl_to_q(joint_map.q_to_ctrl(ctrl))
    assert all(abs(a - b) < 1e-12 for a, b in zip(back, ctrl))


def test_explicit_zero_override_is_used():
    # 运行期标定（节点参数 arm_zero_q）可以覆盖默认零位
    zero = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    q = joint_map.ctrl_to_q([1.0] * 6, zero)
    assert [round(v, 6) for v in q] == [0.9, 0.8, 0.7, 0.6, 0.5, 0.4]
    back = joint_map.q_to_ctrl(q, zero)
    assert all(abs(v - 1.0) < 1e-12 for v in back)


def test_zero_override_length_is_validated():
    with pytest.raises(ValueError):
        joint_map.ctrl_to_q([0.0] * 6, [0.0] * 3)


def test_ctrl_to_q_is_zero_offset_subtraction():
    q = joint_map.ctrl_to_q([0.0, 2.356194490192345, 3.141592653589793, 0.0, 0.0, 0.0])
    assert all(abs(v) < 1e-12 for v in q)


def test_clamp_saturates_to_hardware_limits():
    # 负值截 0；J2 上限 255°；其余 270°
    clamped = joint_map.clamp_cmd([-1.0, 99.0, 99.0, 99.0, 99.0, 99.0])
    assert clamped[0] == 0.0
    assert abs(clamped[1] - 4.45059) < 1e-12
    assert abs(clamped[2] - 4.71238898038469) < 1e-12


def test_clamp_is_idempotent_on_valid_values():
    # Home 指令（0/225/270/0/0/0）本来就在行程内，饱和必须不动它
    valid = joint_map.q_to_ctrl(joint_map.HOME_Q)
    assert joint_map.clamp_cmd(valid) == valid


def test_length_mismatch_is_rejected():
    with pytest.raises(ValueError):
        joint_map.ctrl_to_q([0.0] * 5)
    with pytest.raises(ValueError):
        joint_map.q_to_ctrl([0.0] * 7)


def test_non_finite_is_rejected():
    bad = [0.0, 0.0, 0.0, 0.0, 0.0, math.nan]
    with pytest.raises(ValueError):
        joint_map.ctrl_to_q(bad)
    with pytest.raises(ValueError):
        joint_map.clamp_cmd([math.inf] * 6)


def test_arm_cmd_max_matches_contract_j2_limit():
    # 契约 §1：J2 行程 0..255°，写入 URDF command_interface max = 4.45059
    assert joint_map.ARM_CMD_MAX[1] == 4.45059
    assert all(value == joint_map.ARM_CMD_MIN or value > 0.0 for value in joint_map.ARM_CMD_MAX)
