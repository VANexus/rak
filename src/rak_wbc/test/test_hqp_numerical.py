"""HQP 数值回归：约束激活 / 任务冲突 / warm-start / 降级（spec §9.1）。

[AI] 为什么存在：这层最容易出的错是"约束没生效"和"权重比例搞反"——两者在台架上都
表现为危险行为。这里全部在桌面上用确定性数值断言覆盖。
"""
import numpy as np
import pytest

from conftest import assert_within_limits

from rak_wbc.hqp import HqpSolver, WbcInput


def _nav_input(kin, params, q_arm, v_cmd, **kwargs):
    return WbcInput(
        q_base=np.zeros(3),
        q_arm=np.asarray(q_arm, dtype=float),
        v_cmd=np.asarray(v_cmd, dtype=float),
        mode="nav",
        **kwargs,
    )


def test_idle_mode_returns_zeros(kin, params):
    solver = HqpSolver(kin, params())
    result = solver.solve(
        WbcInput(q_base=np.zeros(3), q_arm=solver.p.home_q, v_cmd=np.zeros(3), mode="idle")
    )
    assert result.status == "idle"
    assert np.allclose(result.v, 0.0)


def test_base_tracking_tracks_cmd_vel(kin, params):
    solver = HqpSolver(kin, params())
    result = solver.solve(_nav_input(kin, params, solver.p.home_q, [0.08, 0.0, 0.0]))
    assert result.status in ("solved", "solved inaccurate")
    assert np.allclose(result.v[:3], [0.08, 0.0, 0.0], atol=0.01)
    assert np.allclose(result.v[3:], 0.0, atol=0.02)  # 已在家位，姿态任务不驱动
    assert_within_limits(kin, solver, kin.q_from(np.zeros(3), solver.p.home_q), result.v)


def test_wheel_domain_caps_translation(kin, params):
    # ω_max = 2 rad/s → 平移上限 ω_max·r ≈ 0.065 m/s，指令 0.3 必须被压住
    solver = HqpSolver(kin, params(wheel_wmax=2.0))
    result = solver.solve(_nav_input(kin, params, solver.p.home_q, [0.3, 0.0, 0.0]))
    wheels = solver.p.wheel_matrix @ result.v[:3]
    # 容差 2e-4：OSQP eps=1e-6 下经 wheel_matrix（≈30 rad/m）放大的残差上界
    assert np.all(np.abs(wheels) <= 2.0 + 2e-4)
    assert 0.06 <= result.v[0] <= 0.065 + 1e-3


def test_joint_barrier_caps_rate_near_limit(kin, params):
    # J2 距上限 0.01 rad，姿态目标越限 → q̇_J2 被屏障压到 k_joint·0.01
    solver = HqpSolver(kin, params())
    p = solver.p
    q_arm = p.home_q.copy()
    q_arm[1] = p.arm_q_max[1] - 0.01
    result = solver.solve(
        _nav_input(kin, params, q_arm, np.zeros(3), posture_target=p.arm_q_max + 0.5)
    )
    assert result.status in ("solved", "solved inaccurate")
    cap = p.k_joint * (p.arm_q_max[1] - q_arm[1])
    assert 0.0 < result.v[4] <= cap + 1e-6
    assert_within_limits(kin, solver, kin.q_from(np.zeros(3), q_arm), result.v)


def test_task_conflict_arbitration_weight_decides_base_borrowing(kin, params):
    """P1 目标需要横移：高权重时借用底盘（P2 不动），低权重时更依赖臂。"""
    q = kin.q_from(np.zeros(3), [0.0, 1.2, 1.2, 0.0, 0.0, 0.0])
    position0, rotation0 = kin.ee_pose(q)
    target = (position0 + np.array([0.0, 0.05, 0.0]), rotation0)

    def solve_with(w_ee):
        solver = HqpSolver(kin, params(w_ee_pos=w_ee))
        return solver.solve(
            WbcInput(
                q_base=kin.base_pose(q),
                q_arm=kin.arm_q(q),
                v_cmd=np.zeros(3),
                mode="servo",
                ee_target=target,
            )
        )

    high = solve_with(100.0)  # 默认档：P1 显著高于 P2
    low = solve_with(1.0)  # 降到与 P2 同量级以下
    assert high.v[1] > 0.005  # 底盘确实参与（加权 QP 的最小范数分配）
    assert low.v[1] < high.v[1]  # 权重更大 → 借用更多


def test_support_cbf_locks_com_when_margin_consumed(kin, params):
    """支撑域吃满（half = margin）时，CoM 的基座系速率必须为 0；宽松时允许。"""
    q_arm = params().home_q
    extended = params().arm_q_max  # 姿态目标远伸，试图把 CoM 推出去

    tight_solver = HqpSolver(kin, params(support_half=[0.02, 0.02], support_margin=0.02))
    tight = tight_solver.solve(
        _nav_input(kin, params, q_arm, np.zeros(3), posture_target=extended)
    )
    q_full = kin.q_from(np.zeros(3), q_arm)
    jacobian = kin.com_base_xy_jacobian(q_full)
    assert np.allclose(jacobian @ tight.v, 0.0, atol=5e-4)

    loose_solver = HqpSolver(kin, params(support_half=[0.5, 0.5]))
    loose = loose_solver.solve(
        _nav_input(kin, params, q_arm, np.zeros(3), posture_target=extended)
    )
    assert np.linalg.norm(jacobian @ loose.v) > 0.01


def test_warm_start_reuses_solution(kin, params):
    solver = HqpSolver(kin, params())
    inp = _nav_input(kin, params, solver.p.home_q, [0.05, 0.02, 0.1])
    first = solver.solve(inp)
    second = solver.solve(inp)
    assert second.iters <= first.iters
    assert np.allclose(first.v, second.v, atol=1e-4)


def test_solver_matches_true_optimum_across_objective_change(kin, params):
    """回归：目标函数在周期之间变化（nav → servo）时求解必须匹配真最优。

    历史缺陷：曾依赖 osqp.update()，但它在 osqp 1.1.3 上静默失效——旧目标函数
    残留，同一问题解出 cost 7.58 的非最优解却报 solved（表现为"无约束激活却不追
    目标"的假收敛）。现为逐周期 setup + warm_start(x, y)。
    """
    solver = HqpSolver(kin, params())
    q = kin.q_from(np.zeros(3), [0.35, 1.283, 1.042, 0.464, 0.427, 0.46])
    position, rotation = kin.ee_pose(q)
    # 先跑一个目标函数不同的周期（nav），再切 servo——正是旧缺陷的触发路径
    solver.solve(
        WbcInput(q_base=kin.base_pose(q), q_arm=kin.arm_q(q), v_cmd=np.zeros(3), mode="nav")
    )
    target = (position + np.array([0.0043, 0.0005, 0.0276]), rotation)
    inp = WbcInput(
        q_base=kin.base_pose(q),
        q_arm=kin.arm_q(q),
        v_cmd=np.zeros(3),
        mode="servo",
        ee_target=target,
    )
    result = solver.solve(inp)
    assert result.status in ("solved", "solved inaccurate")
    q_full = kin.q_from(inp.q_base, inp.q_arm)
    hessian, grad = solver._assemble_objective(inp, q_full, True)
    reference = np.linalg.solve(hessian, -grad)
    assert np.allclose(result.v, reference, atol=2e-3)


def test_degradation_on_solver_failure(kin, params):
    solver = HqpSolver(kin, params())
    inp = _nav_input(kin, params, solver.p.home_q, [0.05, 0.0, 0.0])
    original = solver._osqp_solve

    def boom(_problem):
        raise RuntimeError("injected solver failure")

    solver._osqp_solve = boom
    first = solver.solve(inp)
    assert first.status == "solve_failed"
    assert np.allclose(first.v, 0.0)  # 停轮 + 臂保持
    assert first.fallbacks == 1
    second = solver.solve(inp)
    assert second.fallbacks == 2
    assert solver.consecutive_failures == 2

    solver._osqp_solve = original
    recovered = solver.solve(inp)
    assert recovered.status in ("solved", "solved inaccurate")
    assert solver.consecutive_failures == 0


def test_constraints_hold_over_random_sweep(kin, params):
    rng = np.random.default_rng(7)
    p = params()
    solver = HqpSolver(kin, p)
    for _ in range(25):
        q_arm = rng.uniform(p.arm_q_min + 0.05, p.arm_q_max - 0.05)
        v_cmd = rng.uniform(-1.0, 1.0, 3) * np.array([0.4, 0.4, 0.8])
        mode = str(rng.choice(["nav", "servo"]))
        q_base = np.zeros(3)
        ee_target = None
        if mode == "servo":
            position, _ = kin.ee_pose(kin.q_from(q_base, q_arm))
            ee_target = (position + rng.uniform(-0.05, 0.05, 3), np.eye(3))
        result = solver.solve(
            WbcInput(q_base=q_base, q_arm=q_arm, v_cmd=v_cmd, mode=mode, ee_target=ee_target)
        )
        assert result.status in ("solved", "solved inaccurate")
        assert_within_limits(kin, solver, kin.q_from(q_base, q_arm), result.v)