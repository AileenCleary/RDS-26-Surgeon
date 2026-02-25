import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.analysis.solvers.tensions import solve_tendon_tensions


def test_weighted_tension_solve_prioritizes_main_rows():
    m = build_model()
    q = np.array([0.0, 0.3, 0.5], float)
    tau = np.array([0.0, -1500.0, -650.0], float)

    A = moment_arm_matrix(m, q)

    T_unw = solve_tendon_tensions(m, q, tau, preload=5.0, row_weights=None, reg_w=1e-3)
    r_unw = A @ T_unw - tau

    T_w = solve_tendon_tensions(
        m, q, tau,
        preload=5.0,
        row_weights=np.array([0.1, 1.0, 1.0], float),
        reg_w=1e-3,
    )
    r_w = A @ T_w - tau

    e_unw = float(np.linalg.norm(r_unw[1:]))
    e_w = float(np.linalg.norm(r_w[1:]))
    assert e_w <= e_unw + 1e-6

    assert (T_w >= -1e-9).all()