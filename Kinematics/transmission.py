from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Sequence, Tuple, Literal

import numpy as np 
from scipy.optimize import nnls 

from components import Pulley
from tendon_types import TendonPath
from kinematics import RoboticFingerKinematics
from update_tendon_kinematics import update_config_geometry
from tendon_length import tendon_lengths_all

@dataclass(frozen=True)
class TransmissionModel:
    D: np.ndarray
class TendonTransmission:
    """ 
    """
    def __init__(
            self, 
            *,
            cfg,
            kin: RoboticFingerKinematics,
            pulleys: Dict[str, Pulley], 
            tendons: Dict[str, TendonPath],
            tendon_order: Sequence[str],
            dof_count: int,
            dof_mode: Literal["3", "4"] = "3",
            eps: float = 1e-4,
    ) -> None:
        self.cfg = cfg
        self.kin = kin
        self.pulleys = pulleys
        self.tendons = tendons
        self.tendon_order = list(tendon_order)
        self.dof_count = int(dof_count)
        self.dof_mode = dof_mode
        self.eps = float(eps)

        self.model = TransmissionModel(
            D=np.zeros((self.dof_count, len(self.tendon_order)), dtype=float)
        )

    def lengths(self,
                q: np.ndarray
    ) -> np.ndarray:
        update_config_geometry(cfg=self.cfg, kin=self.kin, q=q, dof_mode=self.dof_mode)  # type: ignore[arg-type]
        #return tendon_lengths_all(self.tendons, self.pulleys, self.tendon_order)
        return tendon_lengths_all(self.tendons, self.pulleys, self.tendon_order, debug=False)
    
    def compute_D(
            self,
            q: np.ndarray
    ) -> np.ndarray:
        q = np.asarray(q, dtype=float).reshape(-1)
        if q.shape[0] != self.dof_count:
            raise ValueError("")
        
        D = np.zeros((self.dof_count, len(self.tendon_order)), dtype=float)
        for j in range(self.dof_count):
            dq = np.zeros_like(q)
            dq[j] = self.eps
            Lp = self.lengths(q + dq)
            Lm = self.lengths(q - dq)
            D[j, :] = (Lp - Lm) / (2.0*self.eps)

        update_config_geometry(cfg=self.cfg, kin=self.kin, q=q, dof_mode=self.dof_mode) # type: ignore[arg-type]
        self.model = TransmissionModel(D=D)
        return D

    def joint_torques_from_tensions(
            self, 
            T: np.ndarray,
    ) -> np.ndarray:
        """ Compute tau = A @ T. """
        T = np.asarray(T, float).reshape(-1)
        return -self.model.D @ T
    
    def tendon_length_rates_from_qdot(
            self, 
            qdot: np.ndarray,
    ) -> np.ndarray:
        qdot = np.asarray(qdot, float).reshape(-1)
        return self.model.D.T @ qdot

    def solve_tensions(
            self, 
            tau_ref: np.ndarray, 
            alpha : float = 0.0,
    ) -> Tuple[np.ndarray, float]:
        """ Solve for (positive) tendon tensions (NNLS) that best achieves the reference joint torques.
        
        Args:
            tau_ref : (3,) reference (command) generalized torques [Splay, MCP, PIPgen] (Nmm).
            alpha   : (>= 0) tension penalty weight [1e-6:1e-3].
        
        Returns:
            T   : (n_tendons,) non-negative tensions (N).
            torque_error_norm   : ||A@T-tau|| (Nmm).
        """
        tau = np.asarray(tau_ref, dtype=float).reshape(-1)
        if tau.shape[0] != self.dof_count:
            raise ValueError("")
        
        A = -self.model.D
        if alpha <= 0.0:
            T, _ = nnls(A, tau)
        else:
            m = A.shape[1]
            A_aug = np.vstack([A, np.sqrt(float(alpha)) * np.eye(m)])
            b_aug = np.concatenate([tau, np.zeros(m)])
            T, _ = nnls(A_aug, b_aug)
        
        torque_err = float(np.linalg.norm(A @ T - tau))
        return T, torque_err