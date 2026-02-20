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
        # --- DEBUG CHECKS: moment arms / rank / "dead" tendons ---
        tendon_names = list(self.tendon_order)

        # Column norms: which tendons are effectively "dead" (near-zero influence)?
        col_norms = np.linalg.norm(D, axis=0)
        print("\n[CHK] D column norms (bigger => tendon affects at least one DOF):")
        for name, nrm in sorted(zip(tendon_names, col_norms), key=lambda x: x[1], reverse=True):
            flag = "  <-- DEAD?" if nrm < 1e-6 else ""
            print(f"    {name:12s} ||D[:,j]||={nrm: .6e}{flag}")

        # Row norms: are you trying to command a DOF that no tendon affects?
        row_norms = np.linalg.norm(D, axis=1)
        print("[CHK] D row norms (bigger => DOF is controllable by some tendon):")
        for i, nrm in enumerate(row_norms):
            flag = "  <-- UNCONTROLLABLE?" if nrm < 1e-6 else ""
            print(f"    row {i}: ||D[i,:]||={nrm: .6e}{flag}")

        # Rank: how many independent directions in torque space you can span
        # (rank(A) == rank(D) since A = -D)
        r = np.linalg.matrix_rank(D, tol=1e-9)
        print(f"[CHK] rank(D)={r} out of dof_count={self.dof_count}, tendons={len(tendon_names)}")

        # Condition number (if rank >= 2): tells you if it's nearly singular
        if min(D.shape) >= 2 and r >= 2:
            svals = np.linalg.svd(D, compute_uv=False)
            # avoid divide-by-zero if smallest is 0
            cond = float(svals[0] / max(svals[-1], 1e-15))
            print(f"[CHK] svd(D): sigma_max={svals[0]:.3e}, sigma_min={svals[-1]:.3e}, cond~{cond:.3e}")
        # --- CHECK: length sensitivity per tendon per DOF (sanity) ---
        # This reuses the Lp/Lm you already computed for each DOF by recomputing once per DOF,
        # but prints per-tendon deltas so you can see which tendon is not changing.
        print("\n[CHK] Per-DOF tendon length deltas (Lp-Lm) (mm):")
        for j in range(self.dof_count):
            dq = np.zeros_like(q)
            dq[j] = self.eps
            Lp = self.lengths(q + dq)
            Lm = self.lengths(q - dq)
            dL = (Lp - Lm)  # (mm)
            # show only the largest few contributors for this DOF
            idx = np.argsort(np.abs(dL))[::-1]
            print(f"    DOF {j}: top |Lp-Lm|:")
            for k in idx[:min(6, len(idx))]:
                print(f"        {tendon_names[k]:12s} dL={dL[k]: .6e}")
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
            w = np.ones(m)
            if "INTERNAL" in self.tendon_order:
                w[self.tendon_order.index("INTERNAL")] = 1e6  # strong penalty
            A_aug = np.vstack([A, np.sqrt(alpha) * np.diag(w)])
            b_aug = np.concatenate([tau, np.zeros(m)])
            T, _ = nnls(A_aug, b_aug)
        
        torque_err = float(np.linalg.norm(A @ T - tau))
        return T, torque_err