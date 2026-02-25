import numpy as np

from rds_finger.model import build_model

def test_tip_jacobian_shapes_and_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)

    from rds_finger.analysis.kinematics.tip_jacobian import tip_jacobian_xyz, joint_torques_from_tip_force

    J = tip_jacobian_xyz(m, q)
    assert J.shape == (3, 3)
    assert np.isfinite(J).all()

    tau = joint_torques_from_tip_force(m, q, np.array([0.0, 0.0, 20.0]))
    assert tau.shape == (3,)
    assert np.isfinite(tau).all()