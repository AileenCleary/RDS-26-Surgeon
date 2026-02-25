import numpy as np
from rds_finger.analysis.loads.feasibility import feasibility_report


def test_feasibility_detects_sign_blocked_row():
    A = np.array([
        [0.0, 0.0, 0.0],
        [-1.0, -2.0, 0.0],
        [ 3.0,  1.0, 0.0],
    ])
    tau = np.array([0.0, -5.0, -4.0])
    T = np.array([1.0, 1.0, 0.0])
    tau_hat = A @ T

    rep = feasibility_report(A, tau, tau_hat)
    rows = {r["row"]: r for r in rep["rows"]}

    assert rows[2]["status"] == "sign_blocked_negative"