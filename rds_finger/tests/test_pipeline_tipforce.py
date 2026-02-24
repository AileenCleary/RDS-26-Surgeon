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

def test_pipeline_tip_force_runs_and_outputs_finite():
    m = build_model()
    q = np.array([0.0, 0.1, 0.2], float)
    F = np.array([0.0, 0.0, 20.0], float)

    from rds_finger.analysis.pipeline import analyze_tip_force

    out = analyze_tip_force(m, q, F, preload=5.0)

    tau = np.asarray(out["tau"], float).reshape(-1)
    A = np.asarray(out["A"], float)
    T = np.asarray(out["tensions"], float).reshape(-1)
    tau_hat = np.asarray(out["tau_hat"], float).reshape(-1)

    # basic sanity
    assert np.isfinite(tau).all()
    assert np.isfinite(A).all()
    assert np.isfinite(T).all()
    assert np.isfinite(tau_hat).all()
    assert (T >= -1e-9).all()

    # internal consistency
    assert np.allclose(tau_hat, A @ T, atol=1e-8)

    # correctness criterion for NNLS in a tendon-driven system:
    # It should not be worse than a trivial baseline.
    err = float(np.linalg.norm(tau_hat - tau))

    # Baseline 1: zero tensions
    tau0 = np.zeros_like(tau)
    err0 = float(np.linalg.norm(tau0 - tau))

    # Baseline 2: preload-only tensions (same preload used by pipeline)
    preload = 5.0
    T_pre = np.ones_like(T) * preload
    tau_pre = A @ T_pre
    err_pre = float(np.linalg.norm(tau_pre - tau))

    # NNLS should be at least as good as the better baseline
    assert err <= min(err0, err_pre) + 1e-8