from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel


def tip_position(model: FingerModel, q: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    Return fingertip position in world coordinates.

    Assumes model.world_state(q) returns: frames, tip_pose, wpulleys, wendpoints, wdrums, wshafts, wbearing
    and that tip_pose.p is the tip position.
    """
    _, tip_pose, *_ = model.world_state(q)
    return np.asarray(tip_pose.p, float).reshape(3)


def tip_jacobian_xyz(
    model: FingerModel,
    q: NDArray[np.float64],
    eps: float = 1e-6,
) -> NDArray[np.float64]:
    """
    Numerical Jacobian J = d x_tip / d q.

    Returns:
        J: shape (3, n_dof)
    """
    q = np.asarray(q, float).reshape(-1)
    x0 = tip_position(model, q)
    n = q.size

    J = np.zeros((3, n), float)
    for j in range(n):
        q2 = q.copy()
        q2[j] += eps
        x1 = tip_position(model, q2)
        J[:, j] = (x1 - x0) / eps
    return J


def joint_torques_from_tip_force(
    model: FingerModel,
    q: NDArray[np.float64],
    F_tip_xyz: NDArray[np.float64],
    eps: float = 1e-6,
) -> NDArray[np.float64]:
    """
    Compute joint torques tau = J^T * F for a tip force.

    Convention:
      - F is expressed in world coordinates
      - tau is in N*mm if your positions are in mm and forces in N.
    """
    F = np.asarray(F_tip_xyz, float).reshape(3)
    J = tip_jacobian_xyz(model, q, eps=eps)
    tau = J.T @ F
    return np.asarray(tau, float).reshape(-1)