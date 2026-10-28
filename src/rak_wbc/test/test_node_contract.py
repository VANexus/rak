"""节点契约的最小检查（需要 rclpy；桌面上自动 skip，Jetson 上运行）。

[AI] 为什么存在：数值层在桌面已全覆盖，但节点参数与默认值的一致性只有在 ROS 环境
才能验证。这里不 spin（不需要 TF/硬件），只做静态契约检查。
"""
import pytest

rclpy = pytest.importorskip("rclpy")

from rak_wbc import node  # noqa: E402


def test_param_defaults_cover_node_contract():
    defaults = node.PARAM_DEFAULTS
    required = {
        "enabled",
        "mode",
        "cmd_vel_topic",
        "ee_target_topic",
        "base_reference_topic",
        "arm_setpoints_topic",
        "publish_arm",
        "home_q",
        "arm_q_min",
        "arm_q_max",
        "arm_qd_max",
        "fail_limit",
    }
    assert required <= set(defaults)


def test_safe_defaults(  # 安全纪律：默认不运动
):
    defaults = node.PARAM_DEFAULTS
    assert defaults["enabled"] is False
    assert defaults["publish_arm"] is False
    assert defaults["mode"] == "idle"


def test_modes_are_closed_set():
    assert set(node._MODES) == {"idle", "nav", "servo", "hold"}