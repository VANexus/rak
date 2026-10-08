"""WBC 组合模型构建与运动学查询（ADR-0011 D3、spec §3）。

[AI] 为什么存在：HQP 需要"底盘 + 臂"的统一运动学，但仓库里臂几何只有一处定义
（`rak_description/urdf/rak_arm.urdf.xacro`）。本模块在运行时把 xacro 展开后的臂
URDF 与本包的平面基座骨架合并为单一 URDF 字符串交给 Pinocchio —— 不复制几何源。

不 import rclpy：数值层可桌面回归（test/）。
"""
from __future__ import annotations

import subprocess
import xml.etree.ElementTree as ET

import numpy as np

ARM_JOINT_NAMES = (
    "base_yaw_joint",
    "shoulder_pitch_joint",
    "elbow_pitch_joint",
    "forearm_roll_joint",
    "wrist_pitch_joint",
    "wrist_roll_joint",
)
PLANAR_JOINT_NAMES = ("wbc_x_joint", "wbc_y_joint", "wbc_yaw_joint")
TOOL_FRAME = "tool0"


def expand_xacro(path: str) -> str:
    """展开 xacro：优先 import xacro，退回 `xacro` CLI（二者要求其一可用）。"""
    try:
        import xacro  # type: ignore
    except ImportError:
        out = subprocess.run(
            ["xacro", str(path)], check=True, capture_output=True, text=True
        )
        return out.stdout
    return xacro.process_file(str(path)).toprettyxml()


def build_composite_urdf(base_xacro: str, arm_xacro: str) -> str:
    """合并 wbc_base（平面链 + 基座 + 臂安装关节）与展开后的臂 URDF。"""
    base_root = ET.fromstring(expand_xacro(base_xacro))
    arm_root = ET.fromstring(expand_xacro(arm_xacro))
    for elem in list(arm_root):
        if elem.tag in ("link", "joint"):
            base_root.append(elem)
    return ET.tostring(base_root, encoding="unicode")


class WbcKinematics:
    """Pinocchio 运动学封装。全部按关节名索引，不依赖 URDF 遍历序。"""

    def __init__(self, urdf_xml: str):
        import pinocchio as pin  # 延迟 import：缺依赖时错误信息清晰

        self._pin = pin
        self.model = pin.buildModelFromXML(urdf_xml)
        self.data = self.model.createData()
        if self.model.nq != 9 or self.model.nv != 9:
            raise ValueError(
                f"expected 9-DOF planar+arm model, got nq={self.model.nq} nv={self.model.nv}"
            )
        self._q_index: dict[str, int] = {}
        for name in PLANAR_JOINT_NAMES + ARM_JOINT_NAMES:
            joint_id = self.model.getJointId(name)
            if joint_id >= len(self.model.joints) or self.model.names[joint_id] != name:
                raise ValueError(f"joint {name!r} not found in composite URDF")
            self._q_index[name] = self.model.joints[joint_id].idx_q
        self._tool_frame = self.model.getFrameId(TOOL_FRAME)
        if self._tool_frame >= len(self.model.frames):
            raise ValueError(f"frame {TOOL_FRAME!r} not found in composite URDF")

    # ---- 装配与积分 --------------------------------------------------------

    def q_from(self, base_pose, q_arm) -> np.ndarray:
        q = np.asarray(self._pin.neutral(self.model), dtype=float)
        for value, name in zip(base_pose, PLANAR_JOINT_NAMES):
            q[self._q_index[name]] = value
        for value, name in zip(q_arm, ARM_JOINT_NAMES):
            q[self._q_index[name]] = value
        return q

    def base_pose(self, q) -> np.ndarray:
        return np.array([q[self._q_index[n]] for n in PLANAR_JOINT_NAMES])

    def arm_q(self, q) -> np.ndarray:
        return np.array([q[self._q_index[n]] for n in ARM_JOINT_NAMES])

    def integrate(self, q, v, dt: float) -> np.ndarray:
        """按 spec §3.2 的机体速度语义积分一步（测试与仿真闭环用）。"""
        x, y, yaw = self.base_pose(q)
        c, s = np.cos(yaw), np.sin(yaw)
        q_next = np.array(q, dtype=float)
        q_next[self._q_index["wbc_x_joint"]] = x + (c * v[0] - s * v[1]) * dt
        q_next[self._q_index["wbc_y_joint"]] = y + (s * v[0] + c * v[1]) * dt
        q_next[self._q_index["wbc_yaw_joint"]] = yaw + v[2] * dt
        for k, name in enumerate(ARM_JOINT_NAMES):
            q_next[self._q_index[name]] += v[3 + k] * dt
        return q_next

    # ---- 末端 --------------------------------------------------------------

    def ee_pose(self, q):
        """返回 (p(3,), R(3,3))，世界系。"""
        self._pin.forwardKinematics(self.model, self.data, q)
        self._pin.updateFramePlacements(self.model, self.data)
        placement = self.data.oMf[self._tool_frame]
        return (
            np.array(placement.translation, dtype=float),
            np.array(placement.rotation, dtype=float),
        )

    def ee_jacobian_body(self, q) -> np.ndarray:
        """6×9：把 QP 变量 v（机体系底盘 + 臂速率）映射到世界对齐的末端 twist。"""
        jac_world = np.array(
            self._pin.computeFrameJacobian(
                self.model, self.data, q, self._tool_frame, self._pin.LOCAL_WORLD_ALIGNED
            ),
            dtype=float,
        )
        yaw = q[self._q_index["wbc_yaw_joint"]]
        c, s = np.cos(yaw), np.sin(yaw)
        to_world = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        transform = np.eye(9)
        transform[0:3, 0:3] = to_world
        return jac_world @ transform

    def rotation_error(self, r_current, r_target) -> np.ndarray:
        """世界系旋转向量误差：log3(R_target · R_currentᵀ)。

        语义 = "把当前姿态转到目标姿态所需的、作用在世界系的旋转向量"，与
        LOCAL_WORLD_ALIGNED 雅可比的角速度行同一坐标系（spec §5）。
        注意不是 log3(R_curᵀR_tgt)——那是体坐标系误差，与本层雅可比不匹配
        （非共轴情形数值不同，桌面测试用混合旋转断言）。
        """
        return np.array(
            self._pin.log3(np.asarray(r_target) @ np.asarray(r_current).T), dtype=float
        )

    # ---- 稳定（spec §3.3） -------------------------------------------------

    def com_base_xy(self, q) -> np.ndarray:
        com_world = np.array(self._pin.centerOfMass(self.model, self.data, q), dtype=float)
        x, y, yaw = self.base_pose(q)
        c, s = np.cos(yaw), np.sin(yaw)
        rot_t = np.array([[c, s], [-s, c]])
        return rot_t @ (com_world[0:2] - np.array([x, y]))

    def com_base_xy_jacobian(self, q) -> np.ndarray:
        """2×9：d(基座系 CoM xy)/dt = J_sup(q)·v（spec §3.3）。

        基座平面运动（平移/偏航）不改变基座系 CoM：平移是刚性跟随；偏航下
        d(Rᵀ)/dψ·r 与 Rᵀ·(dR/dψ)·c_b 两项精确抵消（桌面有限差分测试断言）。
        因此基座列恒为 0，支撑域只在臂构型空间生效。
        """
        jac_world = self._pin.jacobianCenterOfMass(self.model, self.data, q)
        if jac_world is None:
            jac_world = self._pin.computeJacobianCenterOfMass(self.model, self.data, q)
        jac_world = np.array(jac_world, dtype=float)
        yaw = q[self._q_index["wbc_yaw_joint"]]
        c, s = np.cos(yaw), np.sin(yaw)
        rot_t = np.array([[c, s], [-s, c]])
        out = np.zeros((2, 9))
        out[:, 3:9] = rot_t @ jac_world[0:2, 3:9]
        return out