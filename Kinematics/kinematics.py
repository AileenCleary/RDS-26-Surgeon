import numpy as np # pyright: ignore[reportMissingImports]
from numpy.typing import NDArray # pyright: ignore[reportMissingImports]

class RoboticFingerKinematics:
    def __init__(self, 
                 link_lengths, 
                 fingertip_offset : float, 
                 coupling_ratio : float) -> None:
        self.L = np.asarray(link_lengths, dtype=float)
        if self.L.shape[0] != 4:
            raise ValueError("There must be 4 link lengths.")
        self.fingertip_offset = fingertip_offset
        self.k = coupling_ratio

    @staticmethod
    def DH(theta : float, 
           d : float, 
           a : float, 
           alpha : float) -> NDArray[np.float64]:
        c = np.cos(theta); s = np.sin(theta)
        ca = np.cos(alpha); sa = np.sin(alpha)
        return np.array([
            [c, -s*ca,  s*sa, a*c],
            [s,  c*ca, -c*sa, a*s],
            [0,  sa,      ca,   d],
            [0,   0,       0,   1],
        ], dtype=float)
    
    def transforms(self, q):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != 3:
            raise ValueError("q must be shape (3,).")
        
        th0, th1, th2 = q
        T_01 = self.DH(th0, d=0, a=self.L[0], alpha=np.pi/2)
        T_12 = self.DH(th1, d=0, a=self.L[1], alpha=0)
        T_23 = self.DH(th2, d=0, a=self.L[2], alpha=0)
        T_34 = self.DH(th2*self.k, d=0, a=self.L[3], alpha=0)
        T_4F = self.DH(0, d=0, a=self.fingertip_offset, alpha=0)

        T_0F = T_01 @ T_12 @ T_23 @ T_34 @ T_4F
        return {
            "T_01": T_01, "T_12": T_12, "T_23": T_23, "T_34": T_34, "T_4F": T_4F,
            "T_0F": T_0F
        }
    
    def tip_pose(self, q):
        T_0F = self.transforms(q)["T_0F"]
        p = T_0F[0:3, 3]
        R = T_0F[0:3, 0:3]
        return p, R
    
    def jacobian_tip_xy(self, q, eps=1e-6):
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != 3:
            raise ValueError("q must be shape (3,).")
        p0, _ = self.tip_pose(q)
        p0 = p0[:2]

        J = np.zeros((2, 3), dtype=float)
        for i in range(3):
            dq = np.zeros(3)
            dq[i] = eps
            p1, _ = self.tip_pose(q + dq)
            J[:, i] = (p1[:2] - p0) / eps
        return J
    
    def joint_torque_from_tip_force_xy(self, q, F_xy):
        F_xy = np.asarray(F_xy, dtype=float).reshape(-1)
        if F_xy.shape[0] != 2:
            raise ValueError("F_xy must be shape (2,).")
        J = self.jacobian_tip_xy(q)
        return J.T @ F_xy