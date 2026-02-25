import numpy as np

from rds_finger.model import build_model
from rds_finger.routing.router import route_tendons
from rds_finger.loads.pulley_loads import pulley_loads_frictionless

def test_pulley_loads_finite_and_linear_in_T():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(q)

    spec = m.tendons["DIP_FLX"]
    rt = route_tendons(spec, wp, we, wd)

    L1 = pulley_loads_frictionless(rt, tendon_name="DIP_FLX", tension=10.0)
    L2 = pulley_loads_frictionless(rt, tendon_name="DIP_FLX", tension=20.0)

    assert len(L1) == len(L2)
    for a, b in zip(L1, L2):
        assert np.all(np.isfinite(a.F_world))
        assert np.all(np.isfinite(b.F_world))
        assert np.allclose(b.F_world, 2.0 * a.F_world, atol=1e-6)