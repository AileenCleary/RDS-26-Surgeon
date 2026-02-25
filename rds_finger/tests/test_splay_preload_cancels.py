import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix


def test_equal_preload_cancels_splay_torque():
    m = build_model()
    q = np.array([0.0, 0.3, 0.5], float)
    A = moment_arm_matrix(m, q)

    i_row = 0
    iA = m.tendon_order.index("SPLAY_A")
    iB = m.tendon_order.index("SPLAY_B")

    preload = 5.0
    T = np.zeros(len(m.tendon_order), float)
    T[iA] = preload
    T[iB] = preload

    tau_hat = A @ T

    assert abs(float(tau_hat[i_row])) < 1e-6