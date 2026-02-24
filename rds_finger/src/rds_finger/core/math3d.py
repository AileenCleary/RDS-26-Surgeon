from __future__ import annotations
import numpy as np
from typing import Tuple

EPS = 1e-12

def as3(x):
    a = np.asarray(x, dtype=float).reshape(3)
    return a

def unit(v):
    v = as3(v)
    n = float(np.linalg.norm(v))
    if n < EPS:
        raise ValueError("")
    return v / n

def skew(w):
    wx, wy, wz = as3(w)
    return np.array([[0, -wz, wy],
                     [wz, 0, -wx],
                     [-wy, wx, 0]], dtype=float)

def rot_axis_angle(axis, theta):
    a = unit(axis)
    K = skew(a)
    I = np.eye(3)
    return I + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def safe_acos(x):
    return float(np.arccos(clamp(float(x), -1.0, 1.0)))

def orthonormal(axis_hat : np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build right-hand orthonormal basis (e1,e2,e3) where e3 is along axis hat."""
    
    e3 = unit(axis_hat)
    temp = np.array([1.0, 0.0, 0.0], dtype=float) # global x axis
    if abs(float(np.dot(temp, e3))) > 0.9:
        temp = np.array([0.0, 1.0, 0.0])
    e1 = unit(np.cross(e3, temp))
    e2 = np.cross(e3, e1)
    return e1, e2, e3