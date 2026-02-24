import numpy as np
from rds_finger import config
from rds_finger.model import FingerModel
from rds_finger.loads.bearing_loads import compute_all_bearing_loads


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
        bearings=config.BEARINGS,  # NEW
        fingertip_parent_frame="O3",
        fingertip_offset_local=np.array([getattr(config, "FINGERTIP_OFFSET", 0.0), 0.0, 0.0], float),
    )
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