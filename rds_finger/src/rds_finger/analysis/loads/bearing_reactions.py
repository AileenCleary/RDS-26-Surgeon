from __future__ import annotations

from typing import Tuple

import numpy as np

def solve_two_bearing_reactions(
    *,
    F_world: np.ndarray,
    M_world: np.ndarray,
    b1_center: np.ndarray,
    b2_center: np.ndarray,
    shaft_center: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Solve for bearing reaction forces R1, R2 (3D) such that:
        R1 + R2 = -F
        r1×R1 + r2×R2 = -M
    where r_i = (b_i_center - shaft_center).

    Returns:
        (R1, R2) in world coordinates.
    """
    F = np.asarray(F_world, float).reshape(3)
    M = np.asarray(M_world, float).reshape(3)
    r1 = np.asarray(b1_center, float).reshape(3) - np.asarray(shaft_center, float).reshape(3)
    r2 = np.asarray(b2_center, float).reshape(3) - np.asarray(shaft_center, float).reshape(3)

    I = np.eye(3)
    def cross_mat(r: np.ndarray) -> np.ndarray:
        x, y, z = float(r[0]), float(r[1]), float(r[2])
        return np.array([[0, -z, y],
                         [z, 0, -x],
                         [-y, x, 0]], float)

    A = np.block([
        [I, I],
        [cross_mat(r1), cross_mat(r2)],
    ])
    b = np.concatenate([-F, -M], axis=0)

    x, *_ = np.linalg.lstsq(A, b, rcond=None)
    R1 = x[0:3]
    R2 = x[3:6]
    return R1, R2