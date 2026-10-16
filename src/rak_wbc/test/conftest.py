"""共享夹具：组合模型 + 参数工厂。

[AI] 测试自包含：conftest 把包根加入 sys.path，因此
`python3 -m pytest src/rak_wbc/test` 不依赖外部 PYTHONPATH 即可运行；
缺 pinocchio 时依赖它的用例会被 skip（桌面 venv 已实测 `pip install pin` 可用）。
"""
import sys
from pathlib import Path

import numpy as np
import pytest

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[3]
WBC_BASE_XACRO = PACKAGE_ROOT / "urdf" / "wbc_base.urdf.xacro"
ARM_XACRO = REPO_ROOT / "src" / "rak_description" / "urdf" / "rak_arm.urdf.xacro"

if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from rak_wbc.hqp import WbcParams  # noqa: E402

_DEFAULT_PARAMS = dict(
    # [AI] 2026-10-16：改成**指令角域**（与 config/wbc_params.yaml、rak_wbc/node.py 逐值相同）。
    # 这三行原来还是 q_urdf 域的值；2026-10-15 统一角度域后，臂 URDF 的关节值就是
    # 指令角（零位偏移已烘进 joint origin），旧值会让夹具构造出**错的模型姿态**。
    arm_q_min=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],                                     # 全 0°
    arm_q_max=[
        4.71238898038469,   # J1 270°
        4.4505895925855405,  # J2 255°（实测硬件限，不是 270°）
        4.71238898038469,   # J3 270°
        4.71238898038469,   # J4 270°
        4.71238898038469,   # J5 270°
        4.71238898038469,   # J6 270°
    ],
    arm_qd_max=[1.57, 0.5, 0.5, 1.57, 0.5, 1.57],
    home_q=[0.0, 3.9269908169872414, 4.71238898038469, 0.0, 0.0, 0.0],             # 0/225/270/0/0/0°
    base_vmax=[0.25, 0.15, 0.5],
    wheel_radius=0.0325,
    wheel_k=0.15,
    wheel_wmax=10.0,
    support_half=[0.15, 0.15],
    support_margin=0.02,
)


@pytest.fixture(scope="session")
def kin():
    pytest.importorskip("pinocchio")
    from rak_wbc import model_builder

    urdf = model_builder.build_composite_urdf(str(WBC_BASE_XACRO), str(ARM_XACRO))
    return model_builder.WbcKinematics(urdf)


@pytest.fixture
def params():
    def build(**overrides):
        data = dict(_DEFAULT_PARAMS)
        data.update(overrides)
        return WbcParams.from_mapping(data)

    return build


def assert_within_limits(kin, solver, q, v):
    """断言解满足全部硬约束（含支撑域与轮域）。

    容差 2e-4：覆盖 OSQP 在 eps=1e-6 下的绝对残差与约束行量级（wheel_matrix
    ≈30 rad/m）；仍足以捕获逻辑级违例（历史错误约 4e-4~1e-2 量级）。
    """
    matrix, lower, upper = solver.constraints_at(q)
    lhs = matrix @ np.asarray(v, dtype=float)
    margin = float(np.max(lhs - upper))
    assert margin <= 2e-4, f"hard constraint violated by {margin:.2e}"
    assert np.all(lhs >= lower - 2e-4)