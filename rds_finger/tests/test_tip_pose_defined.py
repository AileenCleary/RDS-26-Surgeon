import numpy as np

from rds_finger.model import build_model
from rds_finger.core.frames import Pose

def test_world_state_includes_tip_pose():
    m = build_model()
    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(np.array([0.0, 0.1, 0.2], float))
    assert isinstance(tip_pose, Pose)
    assert np.all(np.isfinite(tip_pose.p))
    assert tip_pose.p.shape == (3,)