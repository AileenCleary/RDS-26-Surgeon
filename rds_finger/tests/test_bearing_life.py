import numpy as np

from rds_finger.model import build_model
from rds_finger.loads.bearing_loads import BearingLoad
from rds_finger.analysis.loads.bearing_life import compute_bearing_lives

def test_bearing_life_monotone_with_load():
    m = build_model()

    bname = next(iter(m.bearings.keys()))
    shaft = m.bearings[bname].shaft

    L1 = [BearingLoad(bearing=bname, shaft=shaft, R_world=np.array([10.0, 0.0, 0.0], float))]
    L2 = [BearingLoad(bearing=bname, shaft=shaft, R_world=np.array([20.0, 0.0, 0.0], float))]

    life1 = compute_bearing_lives(m, L1)[0]
    life2 = compute_bearing_lives(m, L2)[0]

    assert np.isfinite(life1.L10_rev)
    assert np.isfinite(life2.L10_rev)

    p = float(m.bearings[bname].p)
    expected_ratio = (1.0 / 2.0) ** p
    assert abs(life2.L10_rev / life1.L10_rev - expected_ratio) < 1e-9