"""manifest schema 与不变量的纯逻辑测试（不需要 ROS，也不需要 rak_bringup 在场）。

真实清单（`rak_bringup/config/system_manifest.yaml`）的校验放在
`src/rak_bringup/test/test_system_manifest.py` —— 那个测试有权用 ament 索引去找它。
"""

from __future__ import annotations

from copy import deepcopy

import pytest

from rak_lifecycle.manifest import ManifestError, from_dict, load


def base() -> dict:
    return {
        "version": 1,
        "domains": [
            {
                "name": "body",
                "requires": [],
                "components": [
                    {
                        "node": "/robot_state_publisher",
                        "package": "robot_state_publisher",
                        "executable": "robot_state_publisher",
                        "mode": "observed",
                        "owner": "external:chassis_bringup.launch.py",
                        "launch": "chassis_bringup.launch.py",
                        "role": "body_tf_publisher",
                        "provides": [{"topic": "/tf", "kind": "exists"}],
                    }
                ],
            },
            {
                "name": "task",
                "requires": ["body"],
                "components": [
                    {
                        "node": "/task_executor",
                        "package": "rak_task_executor",
                        "executable": "task_executor",
                        "mode": "managed",
                        "owner": "system_manager",
                        "launch": "task.launch.py",
                    }
                ],
            },
        ],
    }


def test_minimal_manifest_orders_domains_by_dependency():
    manifest = from_dict(base())
    assert manifest.order == ("body", "task")


def test_dependents_is_transitive():
    raw = base()
    raw["domains"].append({
        "name": "expression", "requires": ["task"],
        "components": [{
            "node": "/speech_tts", "package": "rak_speech", "executable": "tts_node",
            "mode": "observed", "owner": "external:speech.launch.py",
            "launch": "speech.launch.py",
        }],
    })
    manifest = from_dict(raw)
    assert manifest.dependents_of("body") == frozenset({"task", "expression"})


def test_disabled_domain_stays_out_of_order():
    raw = base()
    raw["domains"][1]["enabled"] = False
    manifest = from_dict(raw)
    assert manifest.order == ("body",)
    assert [d.name for d in manifest.enabled_domains()] == ["body"]


def test_cycle_is_rejected():
    raw = base()
    raw["domains"][0]["requires"] = ["task"]
    with pytest.raises(ManifestError, match="cycle"):
        from_dict(raw)


def test_unknown_requirement_is_rejected():
    raw = base()
    raw["domains"][1]["requires"] = ["navigation"]
    with pytest.raises(ManifestError, match="unknown domains"):
        from_dict(raw)


def test_a_node_cannot_have_two_owners():
    raw = base()
    duplicate = deepcopy(raw["domains"][1]["components"][0])
    raw["domains"][0]["components"].append(duplicate)
    with pytest.raises(ManifestError, match="单一所有者"):
        from_dict(raw)


def _writer(node: str, interface: str, **extra) -> dict:
    component = {
        "node": node, "package": "mecanum_drive_controller", "executable": "mecanum_drive_controller",
        "mode": "managed", "owner": "system_manager", "launch": "task.launch.py",
        "role": "motion_writer", "interface": interface,
    }
    component.update(extra)
    return component


def test_two_motion_writers_on_one_interface_are_rejected():
    raw = base()
    raw["domains"][1]["components"] += [
        _writer("/wbc", "base_twist"),
        _writer("/wbc2", "base_twist", executable="wbc_node"),
    ]
    with pytest.raises(ManifestError, match="base_twist"):
        from_dict(raw)


def test_different_interfaces_may_have_their_own_writer():
    """底盘与臂各有各的唯一写者，互不冲突（ADR-0011 D1 的粒度是接口，不是全局一个）。"""
    raw = base()
    raw["domains"][1]["components"] += [
        _writer("/wbc", "base_twist"),
        _writer("/arm_controller", "arm_position", executable="arm_controller"),
    ]
    from_dict(raw)


def test_disabled_writer_does_not_count_as_a_second_writer():
    """arm_motion_writer:=jtc|wbc 的互斥就是靠这一条成立的。"""
    raw = base()
    raw["domains"][1]["components"] += [
        _writer("/wbc", "base_twist"),
        _writer("/arm_position_controller", "base_twist", enabled=False),
    ]
    from_dict(raw)


def test_two_body_tf_publishers_are_rejected():
    raw = base()
    second = deepcopy(raw["domains"][0]["components"][0])
    second["node"] = "/second_state_publisher"
    raw["domains"][0]["components"].append(second)
    with pytest.raises(ManifestError, match="body TF"):
        from_dict(raw)


def test_managed_component_must_name_a_manager():
    raw = base()
    raw["domains"][0]["components"][0]["mode"] = "managed"
    raw["domains"][0]["components"][0]["owner"] = "external"
    with pytest.raises(ManifestError, match="must name their manager"):
        from_dict(raw)


def test_age_probe_without_threshold_is_rejected():
    raw = base()
    raw["domains"][0]["components"][0]["provides"] = [
        {"topic": "/tf", "kind": "age"}
    ]
    with pytest.raises(ManifestError, match="max_age_ms"):
        from_dict(raw)


def test_only_measured_age_probes_may_escalate():
    raw = base()
    raw["domains"][0]["components"][0]["provides"] = [
        {"topic": "/tf", "kind": "age", "max_age_ms": 100, "basis": "nominal"}
    ]
    manifest = from_dict(raw)
    probe = manifest.domain("body").components[0].provides[0]
    assert not probe.escalation_allowed
    raw["domains"][0]["components"][0]["provides"] = [
        {"topic": "/tf", "kind": "age", "max_age_ms": 100, "basis": "measured"}
    ]
    assert from_dict(raw).domain("body").components[0].provides[0].escalation_allowed


def test_topic_presence_cannot_be_managed():
    raw = base()
    component = raw["domains"][0]["components"][0]
    component["presence"] = "topic"
    component["mode"] = "managed"
    component["owner"] = "system_manager"
    with pytest.raises(ManifestError, match="冲突"):
        from_dict(raw)


def test_topic_presence_needs_a_probe():
    raw = base()
    component = raw["domains"][0]["components"][0]
    component["presence"] = "topic"
    component["provides"] = []
    with pytest.raises(ManifestError, match="provides"):
        from_dict(raw)


def test_unsupported_version_is_rejected():
    raw = base()
    raw["version"] = 2
    with pytest.raises(ManifestError, match="version"):
        from_dict(raw)


def test_missing_manifest_file_names_the_path(tmp_path):
    with pytest.raises(ManifestError, match="not found"):
        load(tmp_path / "nope.yaml")


def test_empty_domain_is_rejected():
    raw = base()
    raw["domains"][1]["components"] = []
    with pytest.raises(ManifestError, match="no components"):
        from_dict(raw)
