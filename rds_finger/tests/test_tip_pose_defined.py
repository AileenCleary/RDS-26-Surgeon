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


def test_world_state_includes_tip_pose():
    m = build_model()
    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(np.array([0.0, 0.1, 0.2], float))
    assert isinstance(tip_pose, Pose)
    assert np.all(np.isfinite(tip_pose.p))
    assert tip_pose.p.shape == (3,)