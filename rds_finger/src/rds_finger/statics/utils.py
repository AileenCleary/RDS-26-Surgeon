"""utils.py: Helper functions for geometric operations and translations."""

import numpy as np
from typing import Tuple

from rds_finger.config import GLOBAL_UP

def unit(
        v: np.ndarray, 
        tol: float = 1e-12,

) -> np.ndarray:
    """Convert input vector v to a unit vector."""
    v = np.asarray(v, dtype=float) # make sure v is a numpy array
    n = np.linalg.norm(v) # calculate length of vector
    
    if n < tol:
        raise ValueError("Zero-length vector.") # if length is basically 0, return an error
    
    return v / n # return v divided by its length ns

def rot90(
        v_2d: np.ndarray,
) -> np.ndarray:
    """Rotate a 2D vector v_2d by +90 degrees to find perpendicular directions when computing tangent points."""
    v_2d = np.asarray(v_2d, dtype=float) # make sure v_2d is a numpy array of floats

    # rotates vector [x,y] by +90 degrees to [-y,x]
    return np.array([-v_2d[1], v_2d[0]], dtype=float)

def plane_basis_with_vertical(
        axis: np.ndarray,
        tol: float = 1e-12
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = unit(axis)
    v = np.asarray(GLOBAL_UP, dtype=float)

    ey = v - np.dot(v, n) * n

    if np.linalg.norm(ey) < tol:
        v = np.array([1.0, 0.0, 0.0], dtype=float)
        ey = v - np.dot(v, n) * n

        if np.linalg.norm(ey) < tol:
            v = np.array([0.0, 1.0, 0.0], dtype=float)
            ey = v - np.dot(v, n) * n

    ey = unit(ey)
    ex = unit(np.cross(ey, n))
    ey = unit(np.cross(n, ex))

    return ex, ey, n

def project_to_plane_coords(
        p3: np.ndarray, # 3d point
        origin3: np.ndarray, # plane origin point in 3D
        ex: np.ndarray, # plane basis vector
        ey: np.ndarray,
) -> np.ndarray:
    """Project a 3D point into 2D coordinates in a local plane basis."""
    r = np.asarray(p3, dtype=float) - np.asarray(origin3, dtype=float)

    # standard coordinate projection where if
    # r = p3 - origin
    # then
    # x = r dot ex, and y = r dot ey
    return np.array([np.dot(r, ex), np.dot(r, ey)], dtype=float)

def lift_from_plane_coords(
        p2, 
        origin3, 
        ex, 
        ey,
):
    """Project 2D point in local plane basis back into 3D."""
    p2 = np.asarray(p2, dtype=float)
    return np.asarray(origin3, dtype=float) + p2[0] * ex + p2[1] * ey

def proj(
        v,
        axis,
):
    axis = unit(axis)
    return np.dot(v, axis) * axis

def rej(
        v,
        axis,
):
    return np.asarray(v, dtype=float) - proj(v, axis)

def shaft_coord(
        p,
        origin,
        axis,
):
    return float(np.dot(np.asarray(p, dtype=float) - np.asarray(origin, dtype=float), unit(axis)))
