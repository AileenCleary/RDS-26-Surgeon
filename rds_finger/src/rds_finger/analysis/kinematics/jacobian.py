from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel
from rds_finger.routing.router import route_tendons
from rds_finger.analysis.routing.length import tendon_total_length


def tendon_lengths(model: FingerModel, q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute tendon total lengths for the model's tendon_order at configuration q."""
    q = np.asarray(q, float).reshape(-1)

    (
        _frames,
        _tip_pose,
        wpulleys,
        wendpoints,
        wdrums,
        _wshafts,
        _wbearings,
    ) = model.world_state(q)

    L: list[float] = []
    for name in model.tendon_order:
        spec = model.tendons[name]
        rt = route_tendons(spec, wpulleys, wendpoints, wdrums)
        L.append(float(tendon_total_length(model, spec, q, rt)))

    return np.asarray(L, dtype=float)

def moment_arm_matrix(
    model: FingerModel,
    q: NDArray[np.float64],
    eps: float = 1e-6,
) -> NDArray[np.float64]:
    """Finite-difference moment arm matrix A(q) for the mapping: tau ≈ A(q) @ T"""
    q = np.asarray(q, float).reshape(-1)
    if q.size != 3:
        raise ValueError(f"Expected q to have length 3 (DOF=3), got {q.size}.")

    eps = float(eps)
    if eps <= 0.0:
        raise ValueError("eps must be positive.")

    L0 = tendon_lengths(model, q)
    m = int(L0.size)
    n = int(q.size)

    dL = np.zeros((m, n), dtype=float)
    for j in range(n):
        q2 = q.copy()
        q2[j] += eps
        L1 = tendon_lengths(model, q2)
        dL[:, j] = (L1 - L0) / eps

    A = -dL.T  # (n_dof, n_tendons)
    return np.asarray(A, dtype=float)