from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np # pyright: ignore[reportMissingImports]
from numpy.typing import NDArray # pyright: ignore[reportMissingImports]


@dataclass(frozen=True)
class TipPose:
    p_xyz: np.ndarray
    R: np.ndarray

class RoboticFingerKinematics:
    def __init__(
            self, 
            link_lengths, 
            fingertip_offset : float, 
            coupling_ratio : float
    ) -> None:
        L = np.asarray(link_lengths, dtype=float).reshape(-1)
        if L.shape[0] != 4:
            raise ValueError("There must be 4 link lengths.")
        self.L = L
        self.fingertip_offset = float(fingertip_offset)
        self.k = float(coupling_ratio)

    @staticmethod
    def DH(
        theta : float, 
        d : float, 
        a : float, 
        alpha : float
    ) -> NDArray[np.float64]:
        c = float(np.cos(theta))
        s = float(np.sin(theta))
        ca = float(np.cos(alpha))
        sa = float(np.sin(alpha))

        return np.array(
            [
                [c, -s*ca,  s*sa, a*c],
                [s,  c*ca, -c*sa, a*s],
                [0.0,  sa,      ca,   d],
                [0.0,   0.0,       0.0,   1.0],
            ], 
            dtype=float,
        )
    
    def transforms(
            self, 
            q: np.ndarray,
    ) -> Dict[str, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != 3:
            raise ValueError("q must be shape (3,).")
        
        th0, th1, th2 = (float(q[0]), float(q[1]), float(q[2]))

        T_01 = self.DH(th0, d=0, a=self.L[0], alpha=np.pi/2)
        T_12 = self.DH(th1, d=0, a=self.L[1], alpha=0)
        T_23 = self.DH(th2, d=0, a=self.L[2], alpha=0)
        T_34 = self.DH(th2*self.k, d=0, a=self.L[3], alpha=0)
        T_4F = self.DH(0, d=0, a=self.fingertip_offset, alpha=0)

        T_0F = T_01 @ T_12 @ T_23 @ T_34 @ T_4F
        return {
            "T_01": T_01, 
            "T_12": T_12, 
            "T_23": T_23, 
            "T_34": T_34, 
            "T_4F": T_4F,
            "T_0F": T_0F,
        }
    
    def tip_pose(
            self, 
            q: np.ndarray,
    ) -> TipPose:
        T_0F = self.transforms(q)["T_0F"]
        p = np.asarray(T_0F[0:3, 3], dtype=float).reshape(3)
        R = np.asarray(T_0F[0:3, 0:3], dtype=float).reshape(3, 3)
        return TipPose(p_xyz=p, R=R)
    
    def jacobian_tip_xyz(self, q, eps=1e-6):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != 3:
            raise ValueError("q must be shape (3,).")
        p0 = self.tip_pose(q).p_xyz

        J = np.zeros((3, 3), dtype=float)
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = float(eps)
            p1 = self.tip_pose(q + dq).p_xyz
            J[:, i] = (p1 - p0) / float(eps)
        return J
    
    def joint_torque_from_tip_force_xyz(self, q, F_xyz):
        F= np.asarray(F_xyz, dtype=float).reshape(3)
        J = self.jacobian_tip_xyz(q)
        return J.T @ F
    
    def jacobian_tip_xy(self, q, eps=1e-6):
        J = self.jacobian_tip_xyz(q, eps=eps)
        return J[0:2, :]
    
    def joint_torque_from_tip_force_xy(self, q, F_xy):
        F = np.asarray(F_xy, dtype=float).reshape(2)
        J = self.jacobian_tip_xy(q)
        return J.T @ F