from __future__ import annotations

import numpy as np

from rds_finger.model import FingerModel
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.analysis.solvers.solve import nnls


def solve_tendon_tensions(
    model: FingerModel,
    q: np.ndarray,
    tau_ref: np.ndarray,
    eps_A: float = 1e-6,
    preload: float = 0.0,
    row_weights: np.ndarray | None = None,
    reg_w: float = 1e-3,
) -> np.ndarray:
    q = np.asarray(q, float).reshape(-1)
    tau = np.asarray(tau_ref, float).reshape(-1)

    A = moment_arm_matrix(model, q, eps=eps_A)  # shape (n_dof, n_tendons)

    if row_weights is not None:
        w = np.asarray(row_weights, float).reshape(-1)
        if w.size != A.shape[0]:
            raise ValueError(f"row_weights size {w.size} must match n_dof {A.shape[0]}")
        W = np.diag(w)
        A_w = W @ A
        tau_w = W @ tau
    else:
        A_w = A
        tau_w = tau

    if preload > 0.0 or reg_w > 0.0:
        T0 = np.ones(A.shape[1], float) * float(preload)
        Wreg = np.sqrt(float(reg_w))
        A_aug = np.vstack([A_w, Wreg * np.eye(A.shape[1])])
        b_aug = np.concatenate([tau_w, Wreg * T0])
        T = nnls(A_aug, b_aug)
    else:
        T = nnls(A_w, tau_w)

    T[T < 0.0] = 0.0
    return T