import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config
from rds_finger.analysis.kinematics.jacobian import tendon_lengths, moment_arm_matrix

def build_model():
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

def test_lengths_finite():
    m = build_model()
    L = tendon_lengths(m, np.array([0.0, 0.1, 0.2]))
    assert np.all(np.isfinite(L))
    assert np.all(L > 0)

def test_A_finite_and_not_all_zero():
    m = build_model()
    A = moment_arm_matrix(m, np.array([0.0, 0.1, 0.2]))
    assert np.all(np.isfinite(A))
    assert np.linalg.norm(A) > 1e-9