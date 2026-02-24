from __future__ import annotations

from typing import List

import numpy as np

from rds_finger.core.math3d import as3, orthonormal, safe_acos


def tangents_point_circle_parallel_axis(
    p: np.ndarray,
    c: np.ndarray,
    r: float,
    axis: np.ndarray,
) -> List[np.ndarray]:
    """Tangent points on a circle from an external point, in 3D."""
    p = as3(p)
    c = as3(c)
    r = float(r)
    if r <= 0.0:
        raise ValueError("Circle radius must be positive.")

    e1, e2, e3 = orthonormal(axis)

    v = p - c
    vx = float(np.dot(v, e1))
    vy = float(np.dot(v, e2))
    d = float(np.hypot(vx, vy))

    eps = 1e-12

    if d < r - eps:
        raise ValueError("Point is inside circle (in-plane); tangents do not exist.")

    if abs(d - r) <= eps:
        if d < eps:
            q2 = np.array([r, 0.0], dtype=float)
        else:
            q2 = (r / d) * np.array([vx, vy], dtype=float)
        q = c + q2[0] * e1 + q2[1] * e2
        return [as3(q)]

    base = float(np.arctan2(vy, vx))
    theta = float(safe_acos(r / d))

    out: List[np.ndarray] = []
    for sgn in (+1.0, -1.0):
        ang = base + sgn * theta
        q2 = np.array([r * np.cos(ang), r * np.sin(ang)], dtype=float)
        q = c + q2[0] * e1 + q2[1] * e2
        out.append(as3(q))

    return out