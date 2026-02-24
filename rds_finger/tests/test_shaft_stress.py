import numpy as np

from rds_finger import config
from rds_finger.model import FingerModel


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
from rds_finger.loads.loads import ShaftLoad
from rds_finger.statics.shaft_stress import compute_shaft_stresses

def test_shaft_stress_scales_with_moment():
    m = build_model()

    sname = next(iter(m.shafts.keys()))

    sl1={"sl1":ShaftLoad(shaft=sname, F_world=np.zeros(3), M_world=np.array([100.0, 0.0, 0.0], float))}
    sl2 = {"sl2":ShaftLoad(shaft=sname, F_world=np.zeros(3), M_world=np.array([250.0, 0.0, 0.0], float))}
    st1 = compute_shaft_stresses(m, sl1)[0]
    st2 = compute_shaft_stresses(m, sl2)[0]

    assert np.isfinite(st1.von_mises_MPa)
    assert np.isfinite(st2.von_mises_MPa)

    # bending stress linear in M
    assert abs(st2.von_mises_MPa / st1.von_mises_MPa - 2.5) < 1e-9