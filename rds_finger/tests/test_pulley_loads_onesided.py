import numpy as np

from rds_finger.model import build_model
from rds_finger.routing.router import route_tendons
from rds_finger.loads.pulley_loads import pulley_loads_frictionless


def test_one_sided_contact_does_not_throw_and_is_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(q)

    spec = m.tendons["SPLAY_A"]
    rt = route_tendons(spec, wp, we, wd)

    loads = pulley_loads_frictionless(rt, tendon_name="SPLAY_A", tension=10.0)
    assert len(loads) >= 1
    for pl in loads:
        assert np.all(np.isfinite(pl.F_world))
        assert pl.F_world.shape == (3,)