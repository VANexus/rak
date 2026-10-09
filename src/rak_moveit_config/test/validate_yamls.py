#!/usr/bin/env python3
"""validate_yamls.py - assert MoveIt config YAMLs parse and have expected keys."""
import sys
from pathlib import Path

import yaml


EXPECTED = {
    "joint_limits.yaml": {"joint_limits"},
    "kinematics.yaml": {"arm_with_base"},
    "ompl_planning.yaml": {"planning_plugins"},
    "fake_controllers.yaml": {"controller_names", "arm_with_base_controller"},
    "pilz_cartesian_limits.yaml": {"cartesian_limits"},
}


def main() -> int:
    cfg = Path(__file__).resolve().parents[1] / "config"
    assert cfg.is_dir(), f"missing config directory: {cfg}"

    for name, expected_keys in EXPECTED.items():
        p = cfg / name
        assert p.is_file(), f"missing config file: {p}"
        with open(p) as f:
            d = yaml.safe_load(f)
        assert d, f"{name}.yaml empty or invalid"
        missing = expected_keys - set(d.keys())
        assert not missing, f"{name}.yaml missing keys: {missing}"
        print(f"OK: {name}.yaml ({len(d)} top-level keys, expected present)")

    print("validate_yamls: ALL OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
