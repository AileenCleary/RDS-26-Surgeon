import numpy as np
from rds_finger.analysis.loads.bearing_reactions import solve_two_bearing_reactions


def test_two_bearing_reactions_satisfy_equilibrium():
    F = np.array([10.0, -5.0, 3.0], float)
    M = np.array([2.0, 1.0, 0.0], float)

    shaft_center = np.zeros(3, float)
    b1 = np.array([0.0, 0.0, -5.0], float)
    b2 = np.array([0.0, 0.0, +5.0], float)

    R1, R2 = solve_two_bearing_reactions(
        F_world=F, M_world=M,
        b1_center=b1, b2_center=b2,
        shaft_center=shaft_center,
    )

    assert np.allclose(R1 + R2 + F, np.zeros(3), atol=1e-7)

    r1 = b1 - shaft_center
    r2 = b2 - shaft_center
    Mhat = np.cross(r1, R1) + np.cross(r2, R2)
    assert np.allclose(Mhat + M, np.zeros(3), atol=1e-7)