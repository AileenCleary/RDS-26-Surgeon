import numpy as np

from rds_finger.model import build_model
from rds_finger.loads.loads import compute_all_pulley_and_shaft_loads

def test_compute_all_pulley_and_shaft_loads_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    tensions = np.ones(len(m.tendon_order), float) * 10.0

    pulley_loads, shaft_loads = compute_all_pulley_and_shaft_loads(m, q, tensions)

    assert len(pulley_loads) >= 0
    for pl in pulley_loads:
        assert np.all(np.isfinite(pl.F_world))
        assert pl.F_world.shape == (3,)

    for sname, sl in shaft_loads.items():
        assert np.all(np.isfinite(sl.F_world))
        assert np.all(np.isfinite(sl.M_world))
        assert sl.F_world.shape == (3,)
        assert sl.M_world.shape == (3,)