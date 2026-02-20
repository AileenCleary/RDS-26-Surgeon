from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Literal

import numpy as np 

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
    ) -> np.ndarray:
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
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> Dict[str, np.ndarray]:
        q = np.asarray(q, dtype=float).reshape(-1)

        if dof_mode == "3":
            if q.shape[0] != 3:
                raise ValueError("q must be shape (3,).")
            th0, th1, th2 = (float(q[0]), float(q[1]), float(q[2]))
            th3 = self.k * th2
        else:
            if q.shape[0] != 4:
                raise ValueError("q must be shape (4,).")
            th0, th1, th2, th3 = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))

        T_01 = self.DH(th0, d=0, a=float(self.L[0]), alpha=np.pi/2)
        T_12 = self.DH(th1, d=0, a=float(self.L[1]), alpha=0.0)
        T_23 = self.DH(th2, d=0, a=float(self.L[2]), alpha=0.0)
        T_34 = self.DH(th3, d=0, a=float(self.L[3]), alpha=0.0)
        T_4F = self.DH(0.0, d=0, a=float(self.fingertip_offset), alpha=0.0)

        T_02 = T_01 @ T_12
        T_03 = T_02 @ T_23
        T_04 = T_03 @ T_34
        T_0F = T_04 @ T_4F

        return {
            "T_01": T_01, 
            "T_12": T_12, 
            "T_23": T_23, 
            "T_34": T_34, 
            "T_4F": T_4F,
            "T_02": T_02,
            "T_03": T_03,
            "T_04": T_04,
            "T_0F": T_0F,
        }
    
    def frame_origins(
            self,
            q: np.ndarray,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> Dict[str, np.ndarray]:
        Ts = self.transforms(q, dof_mode=dof_mode)

        O0 = np.zeros(3, dtype=float)
        O1 = np.asarray(Ts["T_01"][:3, 3], dtype=float)
        O2 = np.asarray(Ts["T_02"][:3, 3], dtype=float)
        O3 = np.asarray(Ts["T_03"][:3, 3], dtype=float)
        O4 = np.asarray(Ts["T_04"][:3, 3], dtype=float)
        OF = np.asarray(Ts["T_0F"][:3, 3], dtype=float)

        return {
            "O0": O0,
            "O1": O1,
            "O2": O2,
            "O3": O3,
            "O4": O4,
            "OF": OF,
        }
        

    def tip_pose(
            self, 
            q: np.ndarray,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> TipPose:
        T_0F = self.transforms(q, dof_mode=dof_mode)["T_0F"]
        p = np.asarray(T_0F[0:3, 3], dtype=float).reshape(3)
        R = np.asarray(T_0F[0:3, 0:3], dtype=float).reshape(3, 3)
        return TipPose(p_xyz=p, R=R)
    
    def jacobian_tip_xyz(
            self, 
            q: np.ndarray, 
            eps=1e-6,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)
        dof = 3 if dof_mode == "3" else 4
        if q.shape[0] != dof:
            raise ValueError("")
        p0 = self.tip_pose(q, dof_mode=dof_mode).p_xyz

        J = np.zeros((3, dof), dtype=float)
        for i in range(dof):
            dq = np.zeros(dof, dtype=float)
            dq[i] = float(eps)
            p1 = self.tip_pose(q + dq, dof_mode=dof_mode).p_xyz
            J[:, i] = (p1 - p0) / float(eps)
        return J
    
    def joint_torque_from_tip_force_xyz(
            self, 
            q: np.ndarray, 
            F_xyz: np.ndarray,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> np.ndarray:
        F= np.asarray(F_xyz, dtype=float).reshape(3)
        J = self.jacobian_tip_xyz(q, dof_mode=dof_mode)
        return J.T @ F
    
    def jacobian_tip_xy(
            self, 
            q: np.ndarray, 
            eps: float = 1e-6,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> np.ndarray:
        J = self.jacobian_tip_xyz(q, eps=eps, dof_mode=dof_mode)
        return J[0:2, :]
    
    def joint_torque_from_tip_force_xy(
            self, 
            q: np.ndarray, 
            F_xy: np.ndarray,
            *,
            dof_mode: Literal["3", "4"] = "3",
    ) -> np.ndarray:
        F = np.asarray(F_xy, dtype=float).reshape(2)
        J = self.jacobian_tip_xy(q, dof_mode=dof_mode)
        return J.T @ F