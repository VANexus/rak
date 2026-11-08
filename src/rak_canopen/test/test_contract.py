"""契约层的离线单测（纯 Python，不需要 ROS、不需要硬件、不需要 Orin 在线）。

这批测试是"今天没有实机也能推进"的那部分价值：明日上台架前，
所有会被总线验证的东西（COB-ID、字节布局、量化范围、速率、活性判据）都必须已经自洽。
"""

from __future__ import annotations

from pathlib import Path

import pytest

from rak_canopen.contract import (
    ContractError,
    command_liveness_ok,
    decode_build_tag,
    load,
    pdo_rate_hz,
)

CONFIG = Path(__file__).resolve().parents[1] / "config"
V4_SOURCE = Path(__file__).resolve().parents[2] / "rak_chassis" / "src" / "can_session.cpp"


@pytest.fixture(scope="module")
def contract():
    return load(CONFIG, v4_source=V4_SOURCE)


# ---------------------------------------------------------------- COB-ID 派生
def test_chassis_cob_ids_match_the_spec_table(contract):
    """派生结果必须等于 [V5] §4 那张表 —— 不是抄表，是拿表来验算生成规则。"""
    chassis = contract.node("chassis")
    assert {p.name: p.cob_id for p in chassis.pdos} == {
        "rpdo1": 0x201, "rpdo2": 0x301,
        "tpdo1": 0x181, "tpdo2": 0x281, "tpdo3": 0x381, "tpdo4": 0x481,
    }
    assert chassis.node_id == 1
    ids = chassis.cob_ids()
    assert ids[0x701] == "chassis.heartbeat"     # [V5] §3 心跳 0x700+ID
    # SDO 方向以"从站的 SDO 服务端"为参照，实测出处是台架工具本身：
    # tools/canopen_bringup.py 往 0x600+node 发请求、从 0x580+node 收应答（读回 "R73"）。
    assert ids[0x581] == "chassis.sdo_srv_tx"    # 服务端→客户端：Jetson 在这里收
    assert ids[0x601] == "chassis.sdo_srv_rx"    # 客户端→服务端：Jetson 往这里发


def test_arm_cob_ids_match_the_spec_table(contract):
    arm = contract.node("arm")
    assert {p.name: p.cob_id for p in arm.pdos} == {
        "rpdo1": 0x202, "rpdo2": 0x302, "rpdo3": 0x402,
        "tpdo1": 0x182, "tpdo2": 0x282, "tpdo3": 0x382, "tpdo4": 0x482,
    }
    assert arm.cob_ids()[0x702] == "arm.heartbeat"


# ---------------------------------------------------------------- 速率推导
def test_rates_derive_from_sync_and_transmission_type(contract):
    """[SPEC]「SYNC 2 ms（500 Hz）」+ 发送类型 → 速率，全部算出来而不是抄进文件。"""
    assert pdo_rate_hz(5, 2.0) == pytest.approx(100.0)
    assert pdo_rate_hz(10, 2.0) == pytest.approx(50.0)
    assert pdo_rate_hz(50, 2.0) == pytest.approx(10.0)
    assert pdo_rate_hz(254, 2.0) is None       # 事件驱动，无固定速率
    assert contract.node("chassis").pdo("tpdo1").rate_hz == pytest.approx(100.0)
    assert contract.node("arm").pdo("tpdo1").rate_hz == pytest.approx(50.0)
    assert contract.node("arm").pdo("tpdo4").rate_hz == pytest.approx(10.0)


# ---------------------------------------------------------------- 字节布局
def test_every_pdo_fits_one_can_frame(contract):
    for node in contract.nodes.values():
        for pdo in node.pdos:
            assert 1 <= pdo.byte_size <= 8, f"{pdo.node}.{pdo.name} = {pdo.byte_size} 字节"
            assert sum(o.bits for o in pdo.objects) == pdo.byte_size * 8


def test_offsets_are_contiguous(contract):
    for node in contract.nodes.values():
        for pdo in node.pdos:
            offset = 0
            for item in pdo.objects:
                assert item.offset_bits == offset, f"{pdo.name}.{item.name} 偏移不连续"
                offset += item.bits


def test_tpdo4_chassis_is_imu_record(contract):
    names = contract.node("chassis").pdo("tpdo4").object_names
    assert names == ("imu_dyaw", "imu_pitch", "imu_roll", "imu_evt", "imu_age")


# ---------------------------------------------------------------- 编解码往返
def test_negative_speed_roundtrip(contract):
    """INT16 0.01 rad/s：负速度的符号必须靠类型判定，不能靠"看起来是负数"。"""
    # RPDO1(速令) 与 TPDO1(实测速) 布局同构，所以用发出去的字节回读接收侧字段。
    payload = contract.encode("chassis", "rpdo1", {
        "w1spd": -12.34, "w2spd": 12.34, "w3spd": 0.0, "w4spd": 3.21})
    assert len(payload) == 8
    decoded = contract.decode("chassis", 0x181, payload)
    assert decoded["w1spd_fb"] == pytest.approx(-12.34, abs=0.01)
    assert decoded["w2spd_fb"] == pytest.approx(12.34, abs=0.01)
    assert decoded["w4spd_fb"] == pytest.approx(3.21, abs=0.01)


def test_wheel_position_is_int32(contract):
    payload = contract.encode("chassis", "tpdo2", {"w1pos": -2000000, "w2pos": 7000})
    decoded = contract.decode("chassis", 0x281, payload)
    assert decoded["w1pos"] == pytest.approx(-2000000)
    assert decoded["w2pos"] == pytest.approx(7000)


def test_speed_beyond_representable_range_raises(contract):
    """因子 0.01 + INT16 ⇒ ±327.67 rad/s。越界要炸，不要静默饱和——
    这正是 v4「量化因子错 10 倍」那类事故的现形方式。"""
    assert contract.node("chassis").pdo("rpdo1").object("w1spd").physical_range[1] == pytest.approx(327.67)
    with pytest.raises(ContractError, match="超出可表示范围"):
        contract.encode("chassis", "rpdo1", {
            "w1spd": 400.0, "w2spd": 0.0, "w3spd": 0.0, "w4spd": 0.0})


def test_arm_negative_angle_raises_instead_of_silently_clamping(contract):
    """域是 UINT16×0.1° ⇒ 可表示 [0, 6553.5]°。

    v4 时代"负角被 Session 静默截 0"是真实事故（臂限位在 q_urdf 域时踩过）。
    现在它必须是显式失败，而不是一个看起来合法的 0.0°。
    """
    obj = contract.node("arm").pdo("rpdo1").object("j1cmd")
    assert obj.physical_range == pytest.approx((0.0, 6553.5))
    with pytest.raises(ContractError, match="超出可表示范围"):
        contract.encode("arm", "rpdo1", {
            "j1cmd": -5.0, "j2cmd": 0.0, "j3cmd": 0.0, "j4cmd": 0.0})


def test_missing_object_is_an_error_not_a_zero(contract):
    with pytest.raises(ContractError, match="缺对象"):
        contract.encode("chassis", "rpdo1", {"w1spd": 1.0})


def test_short_frame_is_rejected(contract):
    with pytest.raises(ContractError, match="需要 8 字节"):
        contract.decode("chassis", 0x181, b"\x00\x01\x02")


# ---------------------------------------------------------------- 身份与活性
def test_build_tag_decode_matches_the_measured_example():
    """[V5] §5 实测 `52 37 33` = "R73"。这条把"读 SDO 判运行版本"钉成可回归的事实。"""
    assert decode_build_tag([0x52, 0x37, 0x33]) == "R73"
    assert decode_build_tag([0x41, 0x31, 0x34]) == "A14"   # 'A'=0x41


def test_liveness_requires_all_three_conditions(contract):
    window = contract.node("chassis").liveness["window_ms"]
    assert command_liveness_ok(seq_changes=1, last_change_s=0.05,
                               enable_bit_set=True, operational=True, window_ms=window)
    assert not command_liveness_ok(seq_changes=1, last_change_s=0.05,
                                   enable_bit_set=True, operational=False, window_ms=window)
    assert not command_liveness_ok(seq_changes=1, last_change_s=0.05,
                                   enable_bit_set=False, operational=True, window_ms=window)
    assert not command_liveness_ok(seq_changes=0, last_change_s=0.05,
                                   enable_bit_set=True, operational=True, window_ms=window)
    assert not command_liveness_ok(seq_changes=5, last_change_s=0.40,
                                   enable_bit_set=True, operational=True, window_ms=window)


# ---------------------------------------------------------------- 混跑门禁
def test_v4_and_v5_collide_so_must_never_share_a_bus(contract):
    """把"禁止同总线混跑 v4/v5"（ADR-0021）变成算术，而不是口号。

    v4 的 ID 从本仓 v4 后端源码里读（不抄第四份表）。
    """
    assert contract.v4_cob_ids, "没读到 v4 帧常量，路径不对就别宣称验证过"
    hits = {cob for cob, _v4, _v5 in contract.collisions()}
    assert 0x201 in hits and 0x202 in hits, sorted(hits)
    for cob, v4_name, v5_name in contract.collisions():
        assert v4_name.startswith("kId"), (cob, v4_name)
        assert "." in v5_name, (cob, v5_name)


def test_manifest_sha_is_recorded(contract):
    """SSOT 副本哈希：明日对表时能证明本仓用的就是那个版本的清单。"""
    assert len(contract.node("chassis").json_sha256) == 64
    assert contract.node("arm").json_sha256 != contract.node("chassis").json_sha256
