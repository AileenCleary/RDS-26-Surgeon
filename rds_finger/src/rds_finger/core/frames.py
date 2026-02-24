from __future__ import annotations
from dataclasses import dataclass
import numpy as np
from .math3d import unit

@dataclass(frozen=True)
class Pose:
    R: np.ndarray
    p: np.ndarray

    @staticmethod
    def I():
        return Pose(R=np.eye(3), p=np.zeros(3))
    
    @staticmethod
    def from_translation(t: np.ndarray) -> "Pose":
        t = np.asarray(t, float).reshape(3)
        R = np.eye(3, dtype=float)
        return Pose(R=R, p=t)
    
    def T(self):
        T = np.eye(4, dtype=float)
        T[:3, :3] = self.R
        T[:3, 3] = self.p
        return T
    
    def apply(self, x):
        x = np.asarray(x, dtype=float).reshape(3)
        return self.R @ x + self.p

    def inv(self):
        Rt = self.R.T
        return Pose(R=Rt, p=-(Rt @ self.p))
    
    def __matmul__(self, other):
        return Pose(R=self.R @ other.R, p=self.R @ other.p + self.p)
    
def pose_from_axis_angle(axis, theta, p=(0,0,0)):
    from .math3d import rot_axis_angle
    return Pose(R=rot_axis_angle(axis, theta), p=np.asarray(p, float).reshape(3))

@dataclass(frozen=True)
class Line3:
    p: np.ndarray
    a: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "p", np.asarray(self.p, float).reshape(3))
        object.__setattr__(self, "a", unit(self.a))