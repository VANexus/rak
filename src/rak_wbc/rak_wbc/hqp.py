"""HQP 任务栈与 OSQP 组装（ADR-0011 D2/D3，spec §4–§7）。

[AI] 为什么存在：底盘与臂的冲突需要单一仲裁点。本模块把 P0 硬约束（盒 / 关节屏障 /
麦轮转速域 / 倾覆支撑域）与 P1–P3 软任务（末端 / 底盘 / 姿态）组装成一个 9 变量加权 QP，
用 OSQP warm-start 求解。结构恒定（约束 42 行 + 任务行靠权重调制），模式切换只改权重。

不 import rclpy / pinocchio：数值层可桌面回归（test/）。
"""
from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

N_VAR = 9
MODES = ("idle", "nav", "servo", "hold")

_TASK_KEYS = (
    "arm_q_min",
    "arm_q_max",
    "arm_qd_max",
    "home_q",
    "base_vmax",
    "support_half",
)
_SCALAR_FLOAT_KEYS = (
    "wheel_wmax",
    "support_margin",
    "w_ee_pos",
    "w_ee_rot",
    "w_base_lin",
    "w_base_yaw",
    "w_posture",
    "regularization",
    "kp_ee",
    "kr_ee",
    "k_post",
    "k_joint",
    "k_support",
    "eps_abs",
    "eps_rel",
)
_SCALAR_INT_KEYS = ("fail_limit", "max_iter")


def default_wheel_matrix(radius: float, k: float) -> np.ndarray:
    """spec §6.3 的 W（4×3，行序 FL/FR/RL/RR）。

    TODO(verify)：符号以 ros2_controllers `mecanum_drive_controller` 源码为准；
    本矩阵在配置中单点定义，核对后一处修正（ADR-0011 待闭合 #7）。
    """
    return np.array(
        [
            [1.0, -1.0, -k],
            [1.0, 1.0, k],
            [1.0, 1.0, -k],
            [1.0, -1.0, k],
        ]
    ) / radius


@dataclass
class WbcParams:
    arm_q_min: np.ndarray
    arm_q_max: np.ndarray
    arm_qd_max: np.ndarray
    home_q: np.ndarray
    base_vmax: np.ndarray
    wheel_matrix: np.ndarray
    wheel_wmax: float
    support_half: np.ndarray
    support_margin: float
    w_ee_pos: float = 100.0
    w_ee_rot: float = 20.0
    w_base_lin: float = 10.0
    w_base_yaw: float = 5.0
    # 档位纪律：P1:P2:P3 ≈ 100:10:0.2（数量级分离是加权 HQP 的优先级表达）。
    # 实测教训：k_post/w_posture 偏大时姿态任务的持续目标速度（~0.6 rad/s）把最优解
    # 推到冗余分支，与 P1 在零空间对抗——闭环误差停滞甚至增长（spec §5 有实测数据）。
    w_posture: float = 0.2
    regularization: float = 1e-3
    kp_ee: float = 2.0
    kr_ee: float = 1.0
    k_post: float = 0.2
    k_joint: float = 3.0
    k_support: float = 1.5
    fail_limit: int = 3
    # 约束行量级较大（wheel_matrix ≈ 1/r ≈ 30 rad/m），eps 1e-4 会留下 ~4e-4 的
    # 绝对残差；安全相关约束收紧到 1e-6（9 变量问题代价可忽略，见 spec §7）。
    eps_abs: float = 1e-6
    eps_rel: float = 1e-6
    max_iter: int = 4000

    @classmethod
    def from_mapping(cls, mapping: dict) -> "WbcParams":
        """从 ROS 参数风格的扁平字典构建；未知键直接报错（拒绝静默漂移）。"""
        data = dict(mapping)
        wheel_matrix = data.pop("wheel_matrix", None)
        if wheel_matrix is None:
            radius = float(data.pop("wheel_radius"))
            k = float(data.pop("wheel_k"))
            wheel_matrix = default_wheel_matrix(radius, k)
        arrays = {key: np.asarray(data.pop(key), dtype=float) for key in _TASK_KEYS}
        # 标量键可缺省：dataclass 默认值即 spec 初值（配置只覆盖关注项）
        scalars = {key: float(data.pop(key)) for key in _SCALAR_FLOAT_KEYS if key in data}
        ints = {key: int(data.pop(key)) for key in _SCALAR_INT_KEYS if key in data}
        if data:
            raise KeyError(f"unknown WbcParams keys: {sorted(data)}")
        return cls(wheel_matrix=np.asarray(wheel_matrix, dtype=float), **arrays, **scalars, **ints)


@dataclass
class WbcInput:
    q_base: np.ndarray  # (3,) x, y, yaw（世界系）
    q_arm: np.ndarray  # (6,)
    v_cmd: np.ndarray  # (3,) 机体系
    mode: str
    ee_target: tuple | None = None  # (p(3,), R(3,3))，世界系
    posture_target: np.ndarray | None = None  # None → home_q


@dataclass
class WbcResult:
    v: np.ndarray
    status: str
    iters: int = 0
    solve_ms: float = 0.0
    support_margin: float = float("inf")
    fallbacks: int = 0
    ee_active: bool = False


class HqpSolver:
    """9 变量加权 QP + 42 行硬约束；维护 OSQP 实例、warm-start 与失败计数。"""

    def __init__(self, kinematics, params: WbcParams):
        import osqp
        from scipy import sparse

        self._osqp = osqp
        self._sparse = sparse
        self._kin = kinematics
        self.p = params
        self._consec_fail = 0
        self._warm_x = None
        self._warm_y = None

    # ---- 组装 --------------------------------------------------------------

    def _assemble_constraints(self, q):
        """C/l/u：行序 = 盒(18) + 关节屏障(12) + 麦轮(8) + 支撑域(4)。"""
        p = self.p
        rows: list[np.ndarray] = []
        upper: list[float] = []

        box_ub = np.concatenate([p.base_vmax, p.arm_qd_max])
        for i in range(N_VAR):
            row = np.zeros(N_VAR)
            row[i] = 1.0
            rows.append(row)
            upper.append(float(box_ub[i]))
            rows.append(-row)
            upper.append(float(box_ub[i]))

        q_arm = self._kin.arm_q(q)
        for i in range(6):
            row = np.zeros(N_VAR)
            row[3 + i] = 1.0
            slack_hi = max(0.0, float(p.arm_q_max[i] - q_arm[i]))
            slack_lo = max(0.0, float(q_arm[i] - p.arm_q_min[i]))
            rows.append(row)
            upper.append(p.k_joint * slack_hi)
            rows.append(-row)
            upper.append(p.k_joint * slack_lo)

        for i in range(4):
            row = np.zeros(N_VAR)
            row[0:3] = p.wheel_matrix[i]
            rows.append(row)
            upper.append(p.wheel_wmax)
            rows.append(-row)
            upper.append(p.wheel_wmax)

        j_sup = self._kin.com_base_xy_jacobian(q)
        com_xy = self._kin.com_base_xy(q)
        half = p.support_half - p.support_margin
        faces = (
            (j_sup[0], half[0] - com_xy[0]),
            (-j_sup[0], half[0] + com_xy[0]),
            (j_sup[1], half[1] - com_xy[1]),
            (-j_sup[1], half[1] + com_xy[1]),
        )
        for jac_row, slack in faces:
            rows.append(np.array(jac_row, dtype=float))
            upper.append(p.k_support * max(0.0, float(slack)))

        matrix = np.vstack(rows)
        ub = np.asarray(upper, dtype=float)
        lb = np.full(ub.shape, -np.inf)
        return matrix, lb, ub

    def _assemble_objective(self, inp: WbcInput, q, ee_active: bool):
        """返回 H = Σw AᵀA + λI 与 grad = −Σw Aᵀb。

        调用方按 OSQP 形式使用：目标 = ½vᵀPv + qᵀv，其中 P = 2H、q = 2·grad
        （保证与 Σw‖Av−b‖² + λ‖v‖² 完全等价；漏掉因子 2 会让所有非零目标的
        增益减半，任务混合比例也会改变——见桌面测试 test_base_tracking 的收敛断言）。
        """
        p = self.p
        hessian = p.regularization * np.eye(N_VAR)
        grad = np.zeros(N_VAR)

        def add_task(matrix, target, weights):
            nonlocal hessian, grad
            hessian = hessian + matrix.T @ (weights[:, None] * matrix)
            grad = grad - matrix.T @ (weights * target)

        if ee_active:
            jacobian = self._kin.ee_jacobian_body(q)
            p_cur, r_cur = self._kin.ee_pose(q)
            p_target, r_target = inp.ee_target
            target = np.concatenate(
                [
                    p.kp_ee * (np.asarray(p_target) - p_cur),
                    p.kr_ee * self._kin.rotation_error(r_cur, r_target),
                ]
            )
            weights = np.array([p.w_ee_pos] * 3 + [p.w_ee_rot] * 3)
            add_task(jacobian, target, weights)

        base_matrix = np.zeros((3, N_VAR))
        base_matrix[0:3, 0:3] = np.eye(3)
        base_cmd = np.zeros(3) if inp.mode == "hold" else np.asarray(inp.v_cmd, dtype=float)
        add_task(base_matrix, base_cmd, np.array([p.w_base_lin, p.w_base_lin, p.w_base_yaw]))

        posture_target = p.home_q if inp.posture_target is None else np.asarray(inp.posture_target, dtype=float)
        posture_matrix = np.zeros((6, N_VAR))
        posture_matrix[:, 3:9] = np.eye(6)
        add_task(
            posture_matrix,
            p.k_post * (posture_target - self._kin.arm_q(q)),
            np.full(6, p.w_posture),
        )
        return hessian, grad

    def _warm_start(self, problem):
        """复用上一周期成功解的原/对偶变量。

        为什么不是 `problem.update()`：osqp 1.1.3 上 `update(P=...)` 会**静默失效**
        （旧目标函数残留，实测同一 QP 得 cost 7.58 的非最优解并报 solved），且 setup
        会剪除显式零，"全模式"结构恒等无法保证。改为每周期 `setup()` + `warm_start(x, y)`：
        永远解精确问题，实测 0.34 ms 均值 / 0.49 ms 峰值（9 变量规模，预算内，见 spec §7）。
        """
        if self._warm_x is None:
            return
        try:
            problem.warm_start(x=self._warm_x, y=self._warm_y)
        except (AttributeError, TypeError):  # 老版本无该 API：退化为冷启动
            pass

    # ---- 求解 --------------------------------------------------------------

    def solve(self, inp: WbcInput) -> WbcResult:
        if inp.mode not in MODES:
            raise ValueError(f"unknown mode {inp.mode!r}; expected one of {MODES}")
        if inp.mode == "idle":
            return WbcResult(v=np.zeros(N_VAR), status="idle")

        q = self._kin.q_from(inp.q_base, inp.q_arm)
        ee_active = inp.mode == "servo" and inp.ee_target is not None
        hessian, grad = self._assemble_objective(inp, q, ee_active)
        matrix, lb, ub = self._assemble_constraints(q)

        start = time.perf_counter()
        try:
            estimator = self._osqp.OSQP()
            estimator.setup(
                P=self._sparse.csc_matrix(2.0 * hessian),
                q=2.0 * grad,
                A=self._sparse.csc_matrix(matrix),
                l=lb,
                u=ub,
                eps_abs=self.p.eps_abs,
                eps_rel=self.p.eps_rel,
                max_iter=self.p.max_iter,
                verbose=False,
            )
            self._warm_start(estimator)
            result = self._osqp_solve(estimator)
        except Exception:  # 求解器异常：降级为零速（停轮 + 臂保持）
            self._consec_fail += 1
            return WbcResult(
                v=np.zeros(N_VAR), status="solve_failed", fallbacks=self._consec_fail,
                ee_active=ee_active,
            )

        solve_ms = (time.perf_counter() - start) * 1e3
        status = str(result.info.status)
        if status not in ("solved", "solved inaccurate"):
            self._consec_fail += 1
            return WbcResult(
                v=np.zeros(N_VAR), status=status, iters=int(result.info.iter),
                solve_ms=solve_ms, fallbacks=self._consec_fail, ee_active=ee_active,
            )

        self._consec_fail = 0
        vector = np.asarray(result.x, dtype=float)
        try:  # 只缓存"成功解"作为下一周期热启动
            self._warm_x = vector
            self._warm_y = np.asarray(result.y, dtype=float)
        except (AttributeError, TypeError):
            self._warm_x = None
            self._warm_y = None
        return WbcResult(
            v=vector,
            status=status,
            iters=int(result.info.iter),
            solve_ms=solve_ms,
            support_margin=self.support_margin(q),
            fallbacks=0,
            ee_active=ee_active,
        )

    def _osqp_solve(self, problem):
        """独立出来便于测试注入失败路径（降级验收）。

        显式 `raise_error=False`：保持"状态字符串驱动降级"（而非异常驱动），
        并兼容 osqp 1.x 已预告的默认值变更；老版本无此参数时退回默认调用。
        """
        try:
            return problem.solve(raise_error=False)
        except TypeError:
            return problem.solve()

    def support_margin(self, q) -> float:
        com_xy = self._kin.com_base_xy(q)
        half = self.p.support_half - self.p.support_margin
        return float(min(half[0] - abs(com_xy[0]), half[1] - abs(com_xy[1])))

    def constraints_at(self, q):
        """公开只读访问：测试与诊断用（C, l, u）。"""
        return self._assemble_constraints(q)

    @property
    def consecutive_failures(self) -> int:
        return self._consec_fail