import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix


def test_splay_pair_is_antagonistic_at_row0():
    m = build_model()
    q = np.array([0.0, 0.3, 0.5], float)
    A = moment_arm_matrix(m, q)
    i_row = 0
    iA = m.tendon_order.index("SPLAY_A")
    iB = m.tendon_order.index("SPLAY_B")

    aA = float(A[i_row, iA])
    aB = float(A[i_row, iB])

    assert abs(aA) > 1e-9
    assert abs(aB) > 1e-9
    assert aA * aB < 0.0