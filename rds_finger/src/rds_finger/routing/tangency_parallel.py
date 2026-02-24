from __future__ import annotations

from typing import List, Tuple

import numpy as np

from rds_finger.core.math3d import unit, safe_acos, as3, orthonormal

def tangents_two_circles_parallel_axis(
    c1: np.ndarray, 
    r1: float,
    c2: np.ndarray, 
    r2: float,
    axis: np.ndarray,
) -> List[Tuple[np.ndarray, np.ndarray]]:
    """Returns 4 sets of tangent points (p1, p2) of a line tangent to both circles.

    Assumptions:
    - Both circles lie in plane normal to `axis`
    - axis is the (shared) normal direction
    - Always returns 4 tangents when circles are separated enough.
    """
    c1 = as3(c1)
    c2 = as3(c2)
    r1 = float(r1)
    r2 = float(r2)

    e1, e2, e3 = orthonormal(axis)
    
    d = c2 - c1
    dx = float(np.dot(d, e1))
    dy = float(np.dot(d, e2))
    D = float(np.hypot(dx, dy))
    if D <= 1e-12:
        raise ValueError("Circle centers coincide in plane; tangents undefined.")

    tangents: List[Tuple[np.ndarray, np.ndarray]] = []
    base = float(np.arctan2(dy, dx))

    for s in (+1.0, -1.0):  
        r2_signed = s * r2

        cos_t = (r1 - r2_signed) / D
        if abs(cos_t) > 1.0:
            continue

        theta = float(safe_acos(cos_t))

        for sign in (+1.0, -1.0):
            ang = base + sign * theta
            
            p1_2 = np.array([r1 * np.cos(ang), r1 * np.sin(ang)], dtype=float)
            # direction from center2 to tangent point on circle2
            p2_2 = np.array([dx + r2_signed * np.cos(ang), dy + r2_signed * np.sin(ang)], dtype=float)

            p1 = c1 + p1_2[0] * e1 + p1_2[1] * e2
            p2 = c1 + p2_2[0] * e1 + p2_2[1] * e2
            tangents.append((as3(p1), as3(p2)))

    if len(tangents) != 4:
        raise ValueError(f"Expected 4 tangents, got {len(tangents)}. Check geometry.")
    
    return tangents