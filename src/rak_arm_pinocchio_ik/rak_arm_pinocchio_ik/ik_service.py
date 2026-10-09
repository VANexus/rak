"""Pinocchio-backed IK math shared by the service node and tests.

Pure ROS-free numerical IK on a 6-DOF arm model loaded from URDF. Algorithm:
damped least-squares (DLS) on the LOCAL_WORLD_ALIGNED frame Jacobian with
per-step joint-limit clamping. No redundancy nullspace projection (the 6-DOF
arm is non-redundant for 6D pose), no collision filtering (those belong to
the planning pipeline, not the per-query solver).

Why DLS rather than a library wrapper:
  * No Python port of pinocchio::utils::Solver ships with ros-jazzy-pinocchio.
  * A handwritten routine keeps the seed -> converge path transparent in
    `test_pinocchio_ik.py`; a library wrapper hides it behind C++.
  * SrvKinematicsPlugin calls this once per IK query, so the per-call budget
    is small. Empirically, 30-80 iterations converge on the six-axis fixture
    at lambda^2 = 1e-4 with step size 0.5.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np
import pinocchio as pin


@dataclass
class IkLimits:
    """Per-joint position limits, indexed parallel to `joint_names`."""

    lower: np.ndarray
    upper: np.ndarray

    def clamp(self, q: np.ndarray) -> np.ndarray:
        return np.minimum(np.maximum(q, self.lower), self.upper)


@dataclass
class IkOptions:
    """Tunables for `solve_ik`. Defaults are sane for a 6-DOF arm."""

    max_iterations: int = 200
    position_tolerance: float = 1e-4  # metres
    rotation_tolerance: float = 1e-3  # radians
    step_size: float = 0.5
    damping: float = 0.01
    seed_neutral_if_missing: bool = True


class PinocchioArmKinematics:
    """Kinematics wrapper for one planning group inside one URDF."""

    def __init__(
        self,
        urdf_xml: str,
        *,
        joint_names: Sequence[str],
        tip_frame: str,
    ) -> None:
        if not joint_names:
            raise ValueError("joint_names must not be empty")
        if not tip_frame:
            raise ValueError("tip_frame must be a non-empty string")

        self._pin = pin
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()

        self.joint_names: tuple[str, ...] = tuple(joint_names)
        self.tip_frame: str = tip_frame

        self._q_index: list[int] = []
        for name in self.joint_names:
            jid = self.model.getJointId(name)
            if jid >= len(self.model.joints):
                raise ValueError(f"joint {name!r} not found in URDF")
            if self.model.names[jid] != name:
                raise ValueError(
                    f"joint lookup mismatch: asked {name!r}, got {self.model.names[jid]!r}"
                )
            self._q_index.append(self.model.joints[jid].idx_q)

        self._tip_frame_id = self.model.getFrameId(tip_frame)
        if self._tip_frame_id >= len(self.model.frames):
            raise ValueError(f"frame {tip_frame!r} not found in URDF")

        big = 1e6
        lower_full = np.array(
            [self.model.lowerPositionLimit[idx] for idx in self._q_index], dtype=float
        )
        upper_full = np.array(
            [self.model.upperPositionLimit[idx] for idx in self._q_index], dtype=float
        )
        lower_full = np.clip(lower_full, -big, big)
        upper_full = np.clip(upper_full, -big, big)
        self.limits = IkLimits(lower=lower_full, upper=upper_full)

    @property
    def n_joints(self) -> int:
        return len(self.joint_names)

    def neutral_q(self) -> np.ndarray:
        q = pin.neutral(self.model)
        return np.array([float(q[i]) for i in self._q_index], dtype=float)

    def forward_kinematics(self, q_arm: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        q = self._full_q_from_arm(q_arm)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self._tip_frame_id]
        return (
            np.array(placement.translation, dtype=float),
            np.array(placement.rotation, dtype=float),
        )

    def frame_jacobian(self, q_arm: np.ndarray) -> np.ndarray:
        q = self._full_q_from_arm(q_arm)
        jac_full = pin.computeFrameJacobian(
            self.model, self.data, q, self._tip_frame_id, pin.LOCAL_WORLD_ALIGNED
        )
        return np.array(jac_full[:, self._q_index], dtype=float)

    def solve_ik(
        self,
        target_position: np.ndarray,
        target_rotation: np.ndarray,
        *,
        seed: np.ndarray | None = None,
        options: IkOptions | None = None,
        position_only: bool = False,
    ) -> tuple[np.ndarray, bool]:
        opts = options or IkOptions()
        q = self._initial_q(seed, opts)
        target_p = np.asarray(target_position, dtype=float).reshape(3)
        target_r = np.asarray(target_rotation, dtype=float).reshape(3, 3)
        q = self.limits.clamp(q)

        for _ in range(opts.max_iterations):
            p_cur, r_cur = self.forward_kinematics(q)
            pos_err = target_p - p_cur
            if position_only:
                rot_err = np.zeros(3)
            else:
                rot_err = np.array(pin.log3(target_r @ r_cur.T), dtype=float)

            if (
                float(np.linalg.norm(pos_err)) < opts.position_tolerance
                and float(np.linalg.norm(rot_err)) < opts.rotation_tolerance
            ):
                return q, True

            j = self.frame_jacobian(q)
            if position_only:
                j = j[:3, :]
                err = pos_err
            else:
                err = np.concatenate([pos_err, rot_err])

            jt = j.T
            gram = j @ jt + (opts.damping ** 2) * np.eye(j.shape[0])
            dq = opts.step_size * jt @ np.linalg.solve(gram, err)
            q = self.limits.clamp(q + dq)

        return q, False

    def _full_q_from_arm(self, q_arm: np.ndarray) -> np.ndarray:
        q_arm = np.asarray(q_arm, dtype=float).reshape(self.n_joints)
        full = pin.neutral(self.model)
        for k, idx in enumerate(self._q_index):
            full[idx] = q_arm[k]
        return full

    def _initial_q(self, seed, opts):
        if seed is None:
            if opts.seed_neutral_if_missing:
                return self.neutral_q()
            raise ValueError("seed is None and seed_neutral_if_missing is False")
        return np.asarray(seed, dtype=float).reshape(self.n_joints)


def load_urdf(path: str) -> str:
    """Read a URDF or xacro file and return URDF XML.

    xacro expansion prefers the Python package if importable, otherwise the
    `xacro` CLI subprocess. Matches the pattern in `rak_wbc/model_builder.py`.
    """
    try:
        import xacro  # type: ignore
    except ImportError:
        import subprocess
        out = subprocess.run(["xacro", path], check=True, capture_output=True, text=True)
        return out.stdout
    return xacro.process_file(path).toprettyxml()


__all__ = ["IkLimits", "IkOptions", "PinocchioArmKinematics", "load_urdf"]
