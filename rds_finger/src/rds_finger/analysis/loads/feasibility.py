from __future__ import annotations

from typing import Dict

import numpy as np

def feasibility_report(
    A: np.ndarray,
    tau: np.ndarray,
    tau_hat: np.ndarray,
    *,
    atol: float = 1e-6,
) -> Dict:
    """
    Light-weight diagnostics for NNLS feasibility.

    If tau component has a sign that no nonnegative combo of columns can produce,
    report that as 'sign-blocked'.

    This is not a full convex cone membership proof, but it catches the common
    tendon-driven failure mode: all contributions in a row have the same sign.

    Returns:
        dict with residual norms and per-row sign feasibility flags.
    """
    A = np.asarray(A, float)
    tau = np.asarray(tau, float).reshape(-1)
    tau_hat = np.asarray(tau_hat, float).reshape(-1)

    r = tau_hat - tau
    row_info = []

    for i in range(A.shape[0]):
        ai = A[i, :]
        nz = np.abs(ai) > atol

        if not np.any(nz):
            row_info.append(
                {"row": i, "status": "unactuated", "tau": float(tau[i]), "tau_hat": float(tau_hat[i])}
            )
            continue

        has_pos = np.any(ai[nz] > 0)
        has_neg = np.any(ai[nz] < 0)

        desired = float(tau[i])
        if abs(desired) <= atol:
            if abs(float(tau_hat[i])) <= 10 * atol:
                status = "ok_zero"
            else:
                status = "zero_desired_but_nonzero_achieved"
        elif desired > 0 and not has_pos:
            status = "sign_blocked_positive"
        elif desired < 0 and not has_neg:
            status = "sign_blocked_negative"
        else:
            status = "not_trivially_blocked"

        row_info.append(
            {
                "row": i,
                "status": status,
                "tau": float(desired),
                "tau_hat": float(tau_hat[i]),
                "resid": float(r[i]),
                "has_pos": bool(has_pos),
                "has_neg": bool(has_neg),
            }
        )

    return {
        "err_norm": float(np.linalg.norm(r)),
        "err_per_row": np.asarray(r, float),
        "rows": row_info,
    }