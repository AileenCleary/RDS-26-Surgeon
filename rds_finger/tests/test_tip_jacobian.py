import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config
from rds_finger.core.frames import Pose


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
        fingertip_parent_frame="O3",
        fingertip_offset_local=np.array([getattr(config, "FINGERTIP_OFFSET", 0.0), 0.0, 0.0], dtype=float),
        bearings=config.BEARINGS
    )

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