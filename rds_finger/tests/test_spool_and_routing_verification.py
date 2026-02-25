import numpy as np

from rds_finger.model import build_model
from rds_finger.routing.router import route_tendons
from rds_finger.routing.types import TendonContact
from rds_finger.analysis.kinematics.jacobian import tendon_lengths, moment_arm_matrix
from rds_finger.analysis.routing.length import tendon_spool_length

def test_verification_spool_router():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], dtype=float)

    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(q)
    for tname in m.tendon_order:
        spec = m.tendons[tname]
        rt = route_tendons(spec, wp, we, wd)

        assert len(rt.points) >= 2, f"{tname}: fewer than 2 points"
        for i in range(len(rt.points) - 1):
            a = rt.points[i]
            b = rt.points[i + 1]
            assert np.all(np.isfinite(a)) and np.all(np.isfinite(b)), f"{tname}: non-finite point"
            d = float(np.linalg.norm(b - a))
            assert d > 1e-9, f"{tname}: zero-length segment at i={i}"

    L = tendon_lengths(m, q)
    assert L.shape == (len(m.tendon_order),)
    assert np.all(np.isfinite(L))
    assert np.all(L > 0.0)

    A = moment_arm_matrix(m, q)  # A = -dL/dq^T
    assert np.all(np.isfinite(A))
    assert np.linalg.norm(A) > 0.0

    any_fixed = False
    for ti, tname in enumerate(m.tendon_order):
        spec = m.tendons[tname]
        last = spec.items[-1]

        if not isinstance(last, TendonContact):
            continue
        if getattr(last, "kind", "idler") != "fixed":
            continue

        any_fixed = True
        pulley = m.pulleys[last.pulley]

        shaft_ref = getattr(pulley, "shaft", None)
        if isinstance(shaft_ref, str):
            assert shaft_ref in m.shafts, f"{tname}: pulley {last.pulley} references missing shaft '{shaft_ref}'"
            shaft = m.shafts[shaft_ref]
        else:
            shaft = shaft_ref

        assert shaft is not None, f"{tname}: pulley {last.pulley} has no shaft"
        assert shaft.dof_row is not None, f"{tname}: fixed pulley {last.pulley} is on dof_row=None shaft"
        j = int(shaft.dof_row)

        eps = 1e-7
        any_fixed = False

        for ti, tname in enumerate(m.tendon_order):
            spec = m.tendons[tname]
            last = spec.items[-1]

            if not isinstance(last, TendonContact):
                continue
            if getattr(last, "kind", "idler") != "fixed":
                continue

            any_fixed = True

            pulley = m.pulleys[last.pulley]
            shaft_ref = getattr(pulley, "shaft", None)
            if isinstance(shaft_ref, str):
                shaft = m.shafts[shaft_ref]
            else:
                shaft = shaft_ref

            assert shaft is not None
            assert shaft.dof_row is not None
            j = int(shaft.dof_row)

            r = float(pulley.radius)
            gain = float(getattr(shaft, "dof_gain", 1.0))
            expected = r * gain

            s0 = tendon_spool_length(m, spec, q)

            q2 = q.copy()
            q2[j] += eps
            s1 = tendon_spool_length(m, spec, q2)

            ds = (s1 - s0) / eps

            spool_sign = float(getattr(last, "spool_sign", 1.0))
            expected_signed = spool_sign * expected
            assert abs(ds - expected_signed) < 1e-4

        assert any_fixed, "No tendons found ending with TendonContact(kind='fixed')."