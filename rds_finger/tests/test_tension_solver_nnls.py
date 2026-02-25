import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.analysis.solvers.tensions import solve_tendon_tensions

def test_solve_tendon_tensions_infeasible_tau_returns_best_fit_nonnegative():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)

    A = moment_arm_matrix(m, q)

    tau = np.array([0.0, 50.0, 25.0], float)

    T = solve_tendon_tensions(m, q, tau)
    assert np.all(np.isfinite(T))
    assert np.all(T >= -1e-12)
    tau_hat = A @ T
    err = float(np.linalg.norm(tau_hat - tau))
    err0 = float(np.linalg.norm(tau))  # A@0 = 0
    assert err <= err0 + 1e-9