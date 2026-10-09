"""组合模型与运动学的桌面数值回归（spec §9.1）。

[AI] 为什么存在：图更复杂（底盘 + 臂 + 稳定量）之后，最先错的是坐标系与雅可比
符号。这里用有限差分对每个解析雅可比做数值对照——不依赖任何硬件。
"""
import numpy as np

from conftest import assert_within_limits


def test_model_structure(kin):
    assert kin.model.nq == 9
    assert kin.model.nv == 9
    q = kin.q_from([0.1, 0.2, 0.3], [0.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    assert np.allclose(kin.base_pose(q), [0.1, 0.2, 0.3])
    assert np.allclose(kin.arm_q(q), [0.0, 1.0, 1.0, 0.0, 0.0, 0.0])
    position, rotation = kin.ee_pose(q)
    assert np.allclose(rotation @ rotation.T, np.eye(3), atol=1e-9)
    assert position[2] > 0.0  # 臂在车顶，tool0 位于 base_link 上方


def test_ee_jacobian_matches_finite_difference(kin):
    q = kin.q_from([0.3, -0.2, 0.4], [0.5, 0.3, -0.4, 0.7, 0.2, 0.6])
    position0, rotation0 = kin.ee_pose(q)
    jacobian = kin.ee_jacobian_body(q)
    eps = 1e-6
    for column in range(9):
        step = np.zeros(9)
        step[column] = 1.0
        position1, rotation1 = kin.ee_pose(kin.integrate(q, step, eps))
        assert np.allclose((position1 - position0) / eps, jacobian[0:3, column], atol=1e-4)
        assert np.allclose(
            kin.rotation_error(rotation0, rotation1) / eps, jacobian[3:6, column], atol=1e-4
        )


def test_com_jacobian_matches_finite_difference(kin):
    q = kin.q_from([0.1, 0.4, -0.7], [0.5, 0.3, -0.4, 0.7, 0.2, 0.6])
    com0 = kin.com_base_xy(q)
    jacobian = kin.com_base_xy_jacobian(q)
    eps = 1e-6
    for column in range(9):
        step = np.zeros(9)
        step[column] = 1.0
        com1 = kin.com_base_xy(kin.integrate(q, step, eps))
        assert np.allclose((com1 - com0) / eps, jacobian[:, column], atol=1e-4)


def test_rotation_error_is_world_frame(kin):
    # 世界系修正 [0.1, 0, 0]（绕 x）施加在已绕 z 转 0.3 的姿态上：世界系误差必须
    # 原样返回该向量；体坐标系误差会被 R_curᵀ 共轭（≈[0.0955, −0.0296, 0]）——
    # 非共轴才能区分两者（这正是本测试的意义）。
    angle = 0.3
    rot_z = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    rot_x = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, np.cos(0.1), -np.sin(0.1)],
            [0.0, np.sin(0.1), np.cos(0.1)],
        ]
    )
    error = kin.rotation_error(rot_z, rot_x @ rot_z)
    assert np.allclose(error, [0.1, 0.0, 0.0], atol=1e-9)


def test_ee_regulation_converges_in_kinematic_loop(kin, params):
    """速度级 resolved-rate 闭环：纯运动学积分下误差应显著收敛（含所有硬约束）。

    起点取中位构型（避开多个关节贴限位）；实测收敛比 ≈0.17（150×10 ms）。
    """
    from rak_wbc.hqp import HqpSolver, WbcInput

    solver = HqpSolver(kin, params())
    q = kin.q_from(np.zeros(3), [0.3, 0.8, 0.9, 0.5, 0.5, 0.5])
    position0, rotation0 = kin.ee_pose(q)
    target = (position0 + np.array([0.03, 0.01, 0.01]), rotation0)
    error0 = float(np.linalg.norm(target[0] - position0))

    dt = 0.01
    for _ in range(150):
        result = solver.solve(
            WbcInput(
                q_base=kin.base_pose(q),
                q_arm=kin.arm_q(q),
                v_cmd=np.zeros(3),
                mode="servo",
                ee_target=target,
            )
        )
        assert result.status in ("solved", "solved inaccurate")
        assert_within_limits(kin, solver, q, result.v)
        q = kin.integrate(q, result.v, dt)

    position1, _ = kin.ee_pose(q)
    error1 = float(np.linalg.norm(target[0] - position1))
    assert error1 < 0.01
    assert error1 < 0.4 * error0