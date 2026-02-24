import numpy as np

from rds_finger.model import build_model
from rds_finger.loads.bearing_loads import compute_all_bearing_loads

def test_bearing_loads_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    tensions = np.ones(len(m.tendon_order), float) * 10.0

    bl = compute_all_bearing_loads(m, q, tensions)
    assert len(bl) > 0
    for x in bl:
        assert np.all(np.isfinite(x.R_world))
        assert x.R_world.shape == (3,)