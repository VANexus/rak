"""EKF 配置的契约测试（ADR-0002 的七条约束 + ADR-0020 的"看得见"要求）。

这里**不**断言融合语义该选哪种（那条有争议，见 ekf.yaml 头部记录），
只钉住"配置本身自洽、且不会静默退化"的部分——全部离线可跑，不需要 ROS 也不需要硬件。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config"
SOURCES = ("odom", "imu")


@pytest.fixture(scope="module")
def params():
    doc = yaml.safe_load((CONFIG / "ekf.yaml").read_text(encoding="utf-8"))
    return doc["ekf_filter_node"]["ros__parameters"]


def _slot_names(params) -> list[str]:
    return sorted({k.split("_")[0] for k in params if "_" in k and
                   any(k.startswith(f"{s}{i}_") for s in SOURCES for i in range(10))})


def test_no_yaml_scientific_notation_strings(params):
    """PyYAML 1.1 把 `1e6` 当**字符串**读，协方差里混进 str 会让 robot_localization
    拿到垃圾数值却不报错。这条坑本仓今天真踩过，必须被门禁钉住。"""
    offenders = [
        (key, index, item)
        for key, value in params.items()
        if isinstance(value, list) and key.endswith("_covariance")
        for index, item in enumerate(value)
        if not isinstance(item, (int, float)) or isinstance(item, bool)
    ]
    assert not offenders, f"非数字元素: {offenders[:3]}"


def test_every_fused_source_declares_its_own_covariance(params):
    """ADR-0002 §决策 4：没有协方差的里程计不要塞进 EKF。"""
    for slot in _slot_names(params):
        config_key, cov_key = f"{slot}_config", f"{slot}_covariance"
        if config_key not in params:
            continue
        used = [flag for flag in params[config_key] if flag]
        if not used:
            continue
        assert cov_key in params, f"{slot} 开了融合却没有 {cov_key}（全靠默认值=假装可信）"
        matrix = params[cov_key]
        assert len(matrix) == 225, f"{slot}_covariance 必须是 15x15，实得 {len(matrix)}"
        diag = [matrix[i * 16] for i in range(15)]
        assert any(d < 1000.0 for d in diag), f"{slot} 全对角都不可信，等于没接这路"


def test_two_d_ground_robot_and_odom_frame_authority(params):
    assert params["two_d_mode"] is True                     # ADR-0002 §约束 7
    assert params["world_frame"] == "odom"                  # 两层结构：EKF 只管 odom→base
    assert params["publish_tf"] is True                     # ADR-0002 §决策 1：EKF 是唯一发布者
    assert params["base_link_frame"] == "base_footprint"    # ADR-0015 REP-120
    assert params["odom_frame"] == "odom"


def test_position_is_fused_by_at_most_one_source(params):
    """两个绝对位姿源会互相打架（ADR-0002 §约束 4 的可执行形式）。"""
    pos_fusers = [
        slot for slot in _slot_names(params)
        if slot in [k.split("_")[0] for k in params if k.endswith("_config")]
        and params[f"{slot}_config"][0] and params[f"{slot}_config"][1]
    ]
    assert len(pos_fusers) <= 1, f"多个源同时提供 x/y 绝对位置: {pos_fusers}"


def test_imu_contributes_only_yaw_rate(params):
    """§约束 1：航向只由 IMU 的 gyro-z 提供，其余通道一律不可信。"""
    config = params["imu0_config"]
    LABELS = ["x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
              "vroll", "vpitch", "vyaw", "ax", "ay", "az"]
    assert config[11] is True, "imu0 必须提供 vyaw，否则航向没有权威源"
    assert sum(1 for flag in config if flag) == 1, "IMU 只应贡献 gyro-z 一路"
    fused = [LABELS[i] for i, flag in enumerate(config) if flag]
    trusted = [LABELS[i] for i in range(15) if params["imu0_covariance"][i * 16] < 1000.0]
    assert fused == trusted, f"config 位与协方差不一致: fused={fused} trusted={trusted}"


def test_diagnostics_are_visible(params):
    """今天现场的失败模式：/odom 没有发布者，EKF 退化成单源，而配置与话题都看不出来。"""
    assert params["print_diagnostics"] is True


def test_wheel_odometry_slot_is_documented_as_not_connected():
    """诚实门禁：轮式里程计（F1 的主力）当前**不在**配置里，因为链路等 P3。
    将来有人接上时，这条测试会强制他同时给协方差与正确的 config 位。"""
    doc = (CONFIG / "ekf.yaml").read_text(encoding="utf-8")
    assert "odom1:" not in doc, "odom1 出现就意味着已接入 —— 必须先补协方差与实测再打开"
    assert "mecanum" in doc.lower() or "轮式" in doc, "必须注明轮式那一路为何缺席"
