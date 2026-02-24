from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.analysis.solvers.solve import nnls, nnls_with_preload  # <- your file name is solve.py


def solve_tendon_tensions(
    model: FingerModel,
    q: NDArray[np.float64],
    tau_ref: NDArray[np.float64],
    eps_A: float = 1e-6,
    preload: float = 0.0,
    row_weights: NDArray[np.float64] | None = None,
    reg_w: float = 1e-3,
) -> NDArray[np.float64]:
    """
    Solve tendon tensions T >= 0 to best match joint torques.

    Baseline solve:
        minimize ||A(q) T - tau_ref|| with T>=0

    With row_weights:
        minimize ||W (A T - tau)|| with T>=0  (W = diag(row_weights))

    With preload + reg_w:
        adds a small regularization to prefer T near a preload vector:
            ||W(A T - tau)||^2 + reg_w ||T - T0||^2
        implemented by augmenting NNLS.

    preload:
        scalar baseline preload applied to ALL tendons in T0.
        (If you later want per-tendon preload, pass a vector instead.)
    """
    q = np.asarray(q, float).reshape(-1)
    tau = np.asarray(tau_ref, float).reshape(-1)

    A = moment_arm_matrix(model, q, eps=eps_A)  # shape (n_dof, n_tendons)

    # Apply row weighting if requested
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
        # Prefer all tendons to sit near preload.
        # This is NOT a hard lower-bound; NNLS still can go below preload if reg_w small.
        # If you want a hard lower bound, that’s a different solver.
        T0 = np.ones(A.shape[1], float) * float(preload)
        # Augment NNLS: [A; sqrt(reg_w) I] T ≈ [tau; sqrt(reg_w) T0]
        Wreg = np.sqrt(float(reg_w))
        A_aug = np.vstack([A_w, Wreg * np.eye(A.shape[1])])
        b_aug = np.concatenate([tau_w, Wreg * T0])
        T = nnls(A_aug, b_aug)
    else:
        T = nnls(A_w, tau_w)

    # Clip numerical negatives
    T[T < 0.0] = 0.0
    return T