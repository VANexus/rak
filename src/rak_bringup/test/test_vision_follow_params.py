"""门禁：`vision_follow_params.yaml` 的 MPPI 半边必须镜像 `nav2_params.yaml`。

`nav2_rotation_shim_controller` 是装饰器：它按**同名节点**（FollowPath）把参数
交给主控制器，所以主控制器（MPPI）的参数没有独立的命名空间。结果就是一份副驾
参数只能在两处出现：`nav2_params.yaml`（给普通 Nav2 用）和
`vision_follow_params.yaml`（给旋转闸门包一层用）。

两份会漂。这个测试就是钉住"不漂"：nav2_params.yaml 里 FollowPath 的每个键（除了
`plugin`，那是被替换的那一项）都必须在 overlay 里有一模一样的值。
"""

import os

import pytest
import yaml

CONFIG_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config")
BASE = os.path.join(CONFIG_DIR, "nav2_params.yaml")
OVERLAY = os.path.join(CONFIG_DIR, "vision_follow_params.yaml")

# Keys the vision stack is ALLOWED to differ in, with a reason. Everything else
# must mirror nav2_params.yaml.
#
# motion_model / vy_*: mecanum strafe was measured not to translate on the
# competition mat (2026-11-19), so the vision stack runs DiffDrive. Plain Nav2
# keeps Omni for the places that still legitimately strafe.
_DELIBERATE = {"plugin", "motion_model", "vy_std", "vy_max", "vy_min"}

_SHIM = "nav2_rotation_shim_controller::RotationShimController"
_MPPI = "nav2_mppi_controller::MPPIController"


def _follow_path(path):
    with open(path, encoding="utf-8") as handle:
        doc = yaml.safe_load(handle)
    return doc["controller_server"]["ros__parameters"]["FollowPath"]


def test_overlay_wraps_mppi_with_the_rotation_shim():
    overlay = _follow_path(OVERLAY)
    assert overlay["plugin"] == _SHIM
    assert overlay["primary_controller"] == _MPPI
    # 视觉路径的 PoseStamped 全是恒等四元数；当成朝向读会得到一堆 0 偏航。
    assert overlay["use_path_orientations"] is False
    assert 0.0 < overlay["angular_disengage_threshold"] < overlay["angular_dist_threshold"]


def test_mppi_parameters_mirror_nav2_params():
    base = _follow_path(BASE)
    overlay = _follow_path(OVERLAY)
    missing = sorted(k for k in base if k not in _DELIBERATE and k not in overlay)
    assert not missing, f"overlay 缺这些 MPPI 键：{missing}"
    drifted = {
        k: (base[k], overlay[k]) for k in base
        if k not in _DELIBERATE and overlay[k] != base[k]
    }
    assert not drifted, f"这两份 MPPI 参数不一致（nav2_params vs overlay）：{drifted}"


def test_base_still_ships_the_plain_mppi_controller():
    # 覆盖层是「加法」：普通 nav2.launch.py 的行为不能被这次改动带走。
    assert _follow_path(BASE)["plugin"] == _MPPI


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-q"]))
