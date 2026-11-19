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


def test_odom_slots_are_contiguous_from_zero(params):
    """robot_localization 从 odom0 起**顺序**探测输入槽，遇到第一个缺失就停。

    2026-11-18 实测：只声明 odom1 时节点根本没这一项参数（`ros2 param list` 只有
    odom0），EKF 静默退化成单 IMU 源，`/odometry/filtered` 照发不误 —— 又一个
    "看着在工作其实没接上"。所以槽位必须从 0 连续。
    """
    index = 0
    while f"odom{index}" in params:
        index += 1
    for later in range(index, index + 4):
        assert f"odom{later}" not in params, (
            f"odom{later} 存在但 odom{index} 缺位：robot_localization 会静默忽略 "
            f"odom{later}（槽位必须从 0 连续）"
        )
    assert index >= 1, "至少要有一路 odom 输入"


def test_wheel_odometry_slot_is_connected_and_honest(params):
    """轮式里程计（F1 的主力）自 2026-11-18 起已接入。

    这条测试原来断言 `odom1` **不存在**（"配置里有槽位不等于已接入"）；接入后它翻转
    成守门：轮式那一路必须只贡献平移，且 yaw/vyaw 必须被排除——麦轮 FK 的 ω 项在
    横向滚子打滑时系统性漂移（实测无稳定器直行 -2.0°/m），航向权威只能是陀螺。
    """
    doc = (CONFIG / "ekf.yaml").read_text(encoding="utf-8")
    assert "odom0:" in doc, "轮式那一路必须接进来（P0-1，且必须是 odom0 槽）"
    assert params["odom0"].startswith("/"), params["odom0"]
    config = params["odom0_config"]
    LABELS = ["x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
              "vroll", "vpitch", "vyaw", "ax", "ay", "az"]
    assert config[0] and config[1], "轮式必须提供 x/y 绝对位置"
    assert config[6], "轮式必须提供 vx"
    for index in (2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14):
        assert not config[index], f"轮式不得贡献 {LABELS[index]}（打滑 + 位置差分不可信）"



def _numeric_sequence_types(params) -> list[tuple[str, int]]:
    """robot_localization 的参数解析器要求同一序列内元素类型一致。

    2026-11-18 实测：协方差里 `0` 与 `0.0025` 混排会让 ekf_node 在启动时
    `Couldn't parse params file: Sequence should be of same type` 直接 abort ——
    整条 SLAM 链没有 odom→base_footprint，现象是"EKF 起了但 TF 没有"。
    PyYAML 把 `0` 读成 int、`0.0025` 读成 float，所以这条门禁看的是**Python 类型**。
    """
    offenders = []
    for key, value in params.items():
        if not isinstance(value, list) or not value:
            continue
        types = {type(item) for item in value if isinstance(item, (int, float))
                 and not isinstance(item, bool)}
        if len(types) > 1:
            offenders.append((key, len(types)))
    return offenders


def test_no_mixed_int_float_sequences(params):
    assert not _numeric_sequence_types(params), (
        f"序列里 int/float 混排会让 ekf_node 启动即 abort: {_numeric_sequence_types(params)}"
    )


def test_default_slot_free_of_declared_but_unused_inputs(params):
    """列了 odom 话题却所有更新量为 false，robot_localization 会每周期报
    "Potentially erroneous data" 把真正的警告淹掉（本轮 /diagnostics 实测被刷屏）。
    不用的输入就别声明。"""
    for slot in ("odom0", "odom1", "odom2", "imu0", "imu1"):
        if slot not in params:
            continue
        assert any(params.get(f"{slot}_config") or []), (
            f"{slot} 声明了话题却没有任何更新量；不用就整块删掉"
        )
