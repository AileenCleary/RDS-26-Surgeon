from __future__ import annotations

import numpy as np

from rds_finger.model import FingerModel


def tip_position(
        model: FingerModel, 
        q: np.ndarray
) -> np.ndarray:
    """Return fingertip position in world frame as shape (3,)."""
    q = np.asarray(q, dtype=float).reshape(-1)
    _, tip, *_ = model.world_state(q)  
    return np.asarray(tip.p, float).reshape(3)


def tip_jacobian_xyz(
        model: FingerModel, 
        q: np.ndarray, 
        eps: float = 1e-6
) -> np.ndarray:
    """
    Finite-difference linear Jacobian J (shape (3, n)) mapping:
        v_tip_world = J(q) @ qdot
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    n = q.size
    if eps <= 0.0:
        raise ValueError("")

    J = np.zeros((3, n), dtype=float)

    for j in range(n):
        dq = np.zeros(n, dtype=float)
        dq[j] = eps

        p1 = tip_position(model, q + dq)
        p0 = tip_position(model, q - dq)
        J[:, j] = (p1 - p0) / (2.0 * eps)

    return J


def tip_force_to_joint_torque(
    model: FingerModel,
    q: np.ndarray,
    F_tip_xyz: np.ndarray,
    eps: float = 1e-6,
) -> np.ndarray:
    """
    Map fingertip force (world frame) to generalized joint torques:
        tau = J(q).T @ F_tip_xyz
    """
    q = np.asarray(q, dtype=float).reshape(-1)
    F = np.asarray(F_tip_xyz, dtype=float).reshape(3)
    J = tip_jacobian_xyz(model, q, eps=eps)
    return (J.T @ F).reshape(-1)