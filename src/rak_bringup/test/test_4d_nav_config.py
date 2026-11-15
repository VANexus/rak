"""4D 融合导航配置契约（ADR-0027 / docs/design/vision_nav_v2.md）。

全部离线可跑：不需要 ROS 图形、不需要传感器、不需要硬件。
钉住的是"配置自洽 + 不会静默退化成 v1"的部分，不是融合效果。
"""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

CONFIG = Path(__file__).resolve().parents[1] / "config"
LABELS = ["x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
          "vroll", "vpitch", "vyaw", "ax", "ay", "az"]


@pytest.fixture(scope="module")
def ekf():
    doc = yaml.safe_load((CONFIG / "ekf_4d.yaml").read_text(encoding="utf-8"))
    return doc["ekf_filter_node"]["ros__parameters"]


@pytest.fixture(scope="module")
def nav2_4d():
    return yaml.safe_load((CONFIG / "nav2_4d.yaml").read_text(encoding="utf-8"))


def _slots(params):
    names = {k.split("_")[0] for k in params if k.endswith("_config")}
    return sorted(names)


def test_four_inputs_are_wired(ekf):
    """轮式 + 激光 + 视觉 + IMU：少一路就不是"用上所有传感器"。"""
    assert _slots(ekf) == ["imu0", "odom0", "odom1", "odom2"], _slots(ekf)
    assert ekf["odom0"] == "/odometry/wheel"
    assert ekf["odom1"] == "/odom"
    assert ekf["odom2"] == "/odometry/vio"
    assert ekf["imu0"] == "/imu/data_raw"


def test_every_input_has_a_dense_covariance(ekf):
    for slot in _slots(ekf):
        used = [flag for flag in ekf[f"{slot}_config"] if flag]
        if not used:
            continue
        matrix = ekf.get(f"{slot}_covariance")
        assert matrix is not None, f"{slot} 没有协方差（ADR-0002 §决策 4）"
        assert len(matrix) == 225, f"{slot} 必须 15x15"
        assert all(isinstance(v, (int, float)) and not isinstance(v, bool) for v in matrix), \
            f"{slot} 里混进了非数字（PyYAML 1.1 会把 1e6 读成字符串）"
        diag = [matrix[i * 16] for i in range(15)]
        assert any(d < 1000.0 for d in diag), f"{slot} 全不可信 = 等于没接"


def test_only_one_absolute_pose_source(ekf):
    """ADR-0002 §约束 4 的可执行形式：绝对位姿只留一路，其余用差分。"""
    absolute = [s for s in _slots(ekf)
                if ekf[f"{s}_config"][0] and ekf[f"{s}_config"][1]
                and not ekf.get(f"{s}_differential", False)]
    assert len(absolute) <= 1, f"多个绝对位姿源会打架: {absolute}"
    assert absolute == ["odom1"], absolute


def test_imu_only_yaw_rate(ekf):
    assert ekf["imu0_config"][11] is True
    assert sum(1 for flag in ekf["imu0_config"] if flag) == 1
    fused = [LABELS[i] for i, flag in enumerate(ekf["imu0_config"]) if flag]
    trusted = [LABELS[i] for i in range(15) if ekf["imu0_covariance"][i * 16] < 1000.0]
    assert fused == trusted, f"config 与协方差口径不一致: {fused} vs {trusted}"


def test_ekf_keeps_the_single_tf_owner(ekf):
    assert ekf["world_frame"] == "odom"
    assert ekf["publish_tf"] is True
    assert ekf["base_link_frame"] == "base_footprint"
    assert ekf["two_d_mode"] is True
    assert ekf["print_diagnostics"] is True


def _voxel_layer(layer):
    assert layer["plugin"] == "rtabmap_costmap_plugins::VoxelLayer"
    assert layer["enabled"] is True
    sources = layer["observation_sources"].split()
    assert "scan" in sources, "雷达是 4D 层唯一的 360° 源"
    assert "depth_cloud" in sources, "深度相机点云是 4D 层的稠密源"
    # 平面雷达 z=0：min_obstacle_height 一旦 > 0 就一个点都不过滤进来（实测过）
    assert layer["scan"]["min_obstacle_height"] == 0.0


def test_local_costmap_uses_4d_voxel_layer(nav2_4d):
    local = nav2_4d["local_costmap"]["local_costmap"]["ros__parameters"]
    assert "voxel_3d_layer" in local["plugins"]
    _voxel_layer(local["voxel_3d_layer"])
    assert local["voxel_3d_layer"]["decay"] > 0, "局部层必须有时间衰减，否则不是 4D"


def test_global_costmap_keeps_static_layer_and_adds_voxels(nav2_4d):
    glob = nav2_4d["global_costmap"]["global_costmap"]["ros__parameters"]
    assert glob["plugins"][0] == "static_layer", "全局层不能丢掉 /map"
    assert "voxel_3d_layer" in glob["plugins"]
    _voxel_layer(glob["voxel_3d_layer"])
    assert glob["voxel_3d_layer"]["decay"] == 0.0, "全局层不衰减，不然慢变障碍会消失"


def test_planner_uses_the_omni_state_lattice(nav2_4d):
    """1.3.13 的 SmacPlannerHybrid 不支持 OMNI（constants.hpp fromString 无该分支），
    原生全向只有 SmacPlannerLattice + share 里的 omni 原语。lattice_filepath 由
    launch 注入，参数文件里不许写死绝对路径。"""
    grid = nav2_4d["planner_server"]["ros__parameters"]["GridBased"]
    assert grid["plugin"] == "nav2_smac_planner::SmacPlannerLattice"
    assert "lattice_filepath" not in grid, "路径归 launch 解析，不要写死"
    text = (CONFIG / "nav2_4d.yaml").read_text(encoding="utf-8")
    assert 'motion_model: "Omni"' not in text
