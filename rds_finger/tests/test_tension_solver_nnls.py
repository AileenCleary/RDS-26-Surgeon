import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.analysis.solvers.tensions import solve_tendon_tensions


def build_model() -> FingerModel:
    return FingerModel(
        link_lengths=config.LINK_LENGTHS,
        coupling_ratio=config.COUPLING_RATIO,
        shafts=config.SHAFTS,
        pulleys=config.PULLEYS,
        drums=config.DRUMS,
        endpoints=config.ENDPOINTS,
        tendons=config.TENDONS,
        tendon_order=config.TENDON_ORDER,
        bearings=config.BEARINGS
    )


def test_solve_tendon_tensions_infeasible_tau_returns_best_fit_nonnegative():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)

    A = moment_arm_matrix(m, q)

    # Pick a torque that is likely infeasible given your sign conventions
    tau = np.array([0.0, 50.0, 25.0], float)

    T = solve_tendon_tensions(m, q, tau)
    assert np.all(np.isfinite(T))
    assert np.all(T >= -1e-12)

    # Basic sanity: NNLS should not do worse than zero tensions.
    tau_hat = A @ T
    err = float(np.linalg.norm(tau_hat - tau))
    err0 = float(np.linalg.norm(tau))  # A@0 = 0
    assert err <= err0 + 1e-9