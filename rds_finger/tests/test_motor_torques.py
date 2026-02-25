import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.loads.motors import compute_drum_torques
from rds_finger.analysis.routing.spool_checks import check_spool_directions

def test_drum_torque_scales_with_tension():
    m = build_model()

    T1 = np.ones(len(m.tendon_order), float) * 10.0
    dt1 = compute_drum_torques(m, T1)
    torques1 = {d.drum: d.torque_Nmm for d in dt1}

    T2 = np.ones(len(m.tendon_order), float) * 25.0
    dt2 = compute_drum_torques(m, T2)
    torques2 = {d.drum: d.torque_Nmm for d in dt2}

    for k in torques1:
        if abs(torques1[k]) < 1e-12:
            continue
        assert abs(torques2[k] / torques1[k] - 2.5) < 1e-9


def test_spool_direction_checks_run():
    m = build_model()
    checks = check_spool_directions(m)
    assert len(checks) == len(m.tendon_order)
    assert all(c.note for c in checks)