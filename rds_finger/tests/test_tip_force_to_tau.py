import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config
from rds_finger.kinematics.jacobian_tip import tip_jacobian_xyz, tip_force_to_joint_torque


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


def test_tip_jacobian_shape_and_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    J = tip_jacobian_xyz(m, q)
    assert J.shape == (3, 3)
    assert np.all(np.isfinite(J))


def test_tip_force_to_tau_linearity():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    F = np.array([0.0, 0.0, 20.0], float)
    tau1 = tip_force_to_joint_torque(m, q, F)
    tau2 = tip_force_to_joint_torque(m, q, 2.0 * F)
    assert np.allclose(tau2, 2.0 * tau1, atol=1e-6)