import numpy as np
from rds_finger import config
from rds_finger.model import FingerModel
from rds_finger.loads.bearing_loads import compute_all_bearing_loads


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
from rds_finger.statics.motors import compute_drum_torques

def test_drum_torque_scales_with_tension():
    m = build_model()

    # all tensions = 10 N
    T1 = np.ones(len(m.tendon_order), float) * 10.0
    dt1 = compute_drum_torques(m, T1)
    torques1 = {d.drum: d.torque_Nmm for d in dt1}

    # all tensions = 25 N
    T2 = np.ones(len(m.tendon_order), float) * 25.0
    dt2 = compute_drum_torques(m, T2)
    torques2 = {d.drum: d.torque_Nmm for d in dt2}

    # torque should scale linearly
    for k in torques1:
        if abs(torques1[k]) < 1e-12:
            # if radius or direction is zero, skip
            continue
        assert abs(torques2[k] / torques1[k] - 2.5) < 1e-9

from rds_finger.statics.spool_checks import check_spool_directions

def test_spool_direction_checks_run():
    m = build_model()
    checks = check_spool_directions(m)
    assert len(checks) == len(m.tendon_order)
    # not forcing all ok because you may intentionally differ
    assert all(c.note for c in checks)