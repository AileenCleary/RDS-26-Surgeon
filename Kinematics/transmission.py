from __future__ import annotations

from dataclasses import dataclass
import enum
from typing import Dict, Sequence, Iterable, Tuple, Union, List

import numpy as np 
from scipy.optimize import nnls 

from components import Pulley
from tendon_types import TendonContact, TendonPath

@dataclass(frozen=True)
class TransmissionModel:
    R: np.ndarray
    D: np.ndarray
    A: np.ndarray
class TendonTransmission:
    """ 
    """
    def __init__(
            self, 
            pulleys: Dict[str, Pulley], 
            tendons: Dict[str, TendonPath],
            tendon_order: Sequence[str],
            D: np.ndarray, 
            *,
            dof_count: int,
            coupling_ratio: float | None = None,
            dip_row: int | None = None,
            pip_row: int | None = None,
            pipgen_row: int | None = None,
    ) -> None:
        self.pulleys = pulleys
        self.tendons = tendons
        self.tendon_order = list(tendon_order)
        self.coupling_ratio = coupling_ratio

        self.D = np.asarray(D, dtype=float)
        if self.D.shape != (dof_count, len(self.tendon_order)):
            raise ValueError("")

        self.dof_count = int(dof_count)
        self.pip_row = pip_row
        self.dip_row = dip_row
        self.pipgen_row = pipgen_row
        
        R = self._build_R_from_paths()
        A = self.D * R
        self.model = TransmissionModel(R=R, D=self.D, A=A)

    def _build_R_from_paths(self) -> np.ndarray:
        """
        Build an R matrix from tendon paths.

        We first build a 'full' R over the physical shaft rows (max dof_row + 1),
        then optionally collapse (PIP, DIP) into a generalized PIP row.
        """
        n = len(self.tendon_order)

        # Determine how many physical rows exist from pulley shaft.dof_row
        dof_rows = [
            p.shaft.dof_row
            for p in self.pulleys.values()
            if p.shaft.dof_row is not None
        ]
        physical_rows = (max(dof_rows) + 1) if dof_rows else self.dof_count
        R_full = np.zeros((physical_rows, n), dtype=float)

        for j, tendon_name in enumerate(self.tendon_order):
            path = self.tendons[tendon_name]

            for elem in path.contacts:
                if not isinstance(elem, TendonContact):
                    continue

                p = self.pulleys[elem.pulley_name]
                row = p.shaft.dof_row
                if row is None:
                    continue
                if row < 0 or row >= physical_rows:
                    raise ValueError(f"{p.name}: dof_row={row} out of bounds")

                R_full[row, j] += float(elem.sign)*float(p.radius)

        # If you want a 3-DOF generalized system (Splay, MCP, PIPgen)
        if (
            self.dof_count == 3
            and self.coupling_ratio is not None
            and self.pip_row is not None
            and self.dip_row is not None
            and self.pipgen_row is not None
        ):
            return self._generalize_R(R_full)

        # Otherwise, return (or truncate) to requested dof_count
        if R_full.shape[0] != self.dof_count:
            if R_full.shape[0] < self.dof_count:
                raise ValueError("Not enough physical dof rows to fill requested dof_count.")
            return R_full[: self.dof_count, :]
        return R_full


    def _generalize_R(self, R_full: np.ndarray) -> np.ndarray:
        """
        Collapse PIP and DIP rows into a generalized PIP row:
            R_pipgen = R_pip + k * R_dip
        """
        k = float(self.coupling_ratio)

        Rg = np.zeros((3, R_full.shape[1]), dtype=float)
        Rg[0, :] = R_full[0, :]  # splay
        Rg[1, :] = R_full[1, :]  # MCP
        Rg[2, :] = R_full[self.pip_row, :] + k * R_full[self.dip_row, :]
        return Rg

    # def _build_R_from_paths(self) -> np.ndarray:
    #     R = np.zeros((self.dof_count, len(self.tendon_order)), dtype=float)

    #     for j, tendon_name in enumerate(self.tendon_order):
    #         path = self.tendons[tendon_name]
    #         if path is None:
    #             raise KeyError("")

    #         for elem in path.contacts:
    #             if not isinstance(elem, TendonContact):
    #                 continue

    #             p = self.pulleys[elem.pulley_name] # type: ignore
    #             if p is None:
    #                 raise KeyError("")
    #             row = getattr(p.shaft, "dof_row", None)
    #             if row is None:
    #                 continue
                
    #             row = int(row)
    #             if row < 0 or row >= self.dof_count:
    #                 raise ValueError("")
                
    #             R[row, j] += float(p.radius)
            
    #     if (
    #         self.dof_count == 3
    #         and self.coupling_ratio is not None
    #         and self.pip_row is not None
    #         and self.dip_row is not None
    #         and self.pipgen_row is not None
    #     ):
    #         R = self._generalize_R(R)
            
    #     return R
                
    # def _build_R4(
    #         self, 
    # ) -> np.ndarray:
    #     n = len(self.tendon_order)
    #     R4 = np.zeros((4, n), dtype=float)

    #     for j, tendon_name in enumerate(self.tendon_order):
    #         path = self.tendon_paths.get(tendon_name, None)
    #         if path is None:
    #             raise ValueError("")
            
    #         for elem in path.contacts:
    #             if not hasattr(elem, "pulley_name"):
    #                 continue

    #             pulley_name = getattr(elem, "pulley_name")
    #             p = self.pulleys[pulley_name]

    #             row = p.shaft.dof_row
    #             if row is None:
    #                 continue

    #             R4[row, j] += float(p.radius)
        
    #     return R4
    
    # def _generalize_R(
    #         self,
    #         R4: np.ndarray,
    # ) -> np.ndarray:
    #     k = float(self.coupling_ratio)
    #     pip = int(self.pip_row)
    #     dip = int(self.dip_row)
    #     pipgen = int(self.pipgen_row)

    #     R = R4.copy()
    #     R[pipgen, :] = R4[pip, :] + k*R4[dip, :]
    #     if pipgen != pip:
    #         R[pip, :] = 0.0
    #     if pipgen != dip:
    #         R[dip, :] = 0.0
    #     return R
    
    
    def joint_torques_from_tensions(
            self, 
            T: np.ndarray,
    ) -> np.ndarray:
        """ Compute tau = A @ T. """
        T = np.asarray(T, dtype=float).reshape(-1)
        return self.model.A @ T
    
    def tendon_length_rates_from_qdot(
            self, 
            qdot: np.ndarray,
    ) -> np.ndarray:
        qdot = np.asarray(qdot, dtype=float).reshape(-1)
        return -(self.model.A.T @ qdot)

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
        
        if alpha <= 0.0:
            T, _ = nnls(self.model.A, tau)
        else:
            m = self.model.A.shape[1]
            A_aug = np.vstack([self.model.A, np.sqrt(float(alpha)) * np.eye(m)])
            b_aug = np.concatenate([tau, np.zeros(m)])
            T, _ = nnls(A_aug, b_aug)
        
        torque_err = float(np.linalg.norm(self.model.A @ T - tau))
        return T, torque_err