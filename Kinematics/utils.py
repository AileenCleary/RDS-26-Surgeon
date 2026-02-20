from __future__ import annotations
from typing import Tuple

import numpy as np # type: ignore

def _unit(v : np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("Vector too close to zero.")
    return v / n

def _angle(u : np.ndarray, v : np.ndarray) -> float:
    """Angle between vectors (rads), in range [0,pi]."""

    u = _unit(u)
    v = _unit(v)
    c = float(np.clip(np.dot(u,v), -1.0, 1.0))
    return float(np.arccos(c))

def _orthonormal(axis_hat : np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build right-hand orthonormal basis (e1,e2,e3) where e3 is along axis hat."""
    
    e3 = _unit(axis_hat)
    temp = np.array([1.0, 0.0, 0.0], dtype=float) # global x axis
    if abs(float(np.dot(temp, e3))) > 0.9:
        temp = np.array([0.0, 1.0, 0.0])
    e1 = _unit(np.cross(e3, temp))
    e2 = np.cross(e3, e1)
    return e1, e2, e3

def _axis_equal(a, b, atol: float = 1e-9):
    a = _unit(a)
    b = _unit(b)
    return bool(np.allclose(a, b, atol))

def _capstan_ratio(
        mu : float, 
        wrap_angle_rad : float
    ) -> float:
    """Capstan equation ratio."""
    return float(np.exp(mu * wrap_angle_rad)) 
