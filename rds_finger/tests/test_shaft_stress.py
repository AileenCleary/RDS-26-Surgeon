import numpy as np

from rds_finger.model import build_model
from rds_finger.loads.loads import ShaftLoad
from rds_finger.analysis.loads.shaft_stress import compute_shaft_stresses

def test_shaft_stress_scales_with_moment():
    m = build_model()

    sname = next(iter(m.shafts.keys()))

    sl1={"sl1":ShaftLoad(shaft=sname, F_world=np.zeros(3), M_world=np.array([100.0, 0.0, 0.0], float))}
    sl2 = {"sl2":ShaftLoad(shaft=sname, F_world=np.zeros(3), M_world=np.array([250.0, 0.0, 0.0], float))}
    st1 = compute_shaft_stresses(m, sl1)[0]
    st2 = compute_shaft_stresses(m, sl2)[0]

    assert np.isfinite(st1.von_mises_MPa)
    assert np.isfinite(st2.von_mises_MPa)

    assert abs(st2.von_mises_MPa / st1.von_mises_MPa - 2.5) < 1e-9