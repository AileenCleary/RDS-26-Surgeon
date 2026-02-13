import numpy as np # pyright: ignore[reportMissingImports]
from scipy.optimize import nnls

class TendonTransmission:
    """ Tendon transmission model for coupled PIP/DIP finger config in generalized coordinates. 
    
    Parameters:
        tau : (3,) generalized joint torques [Splay, MCP, PIPgen] (Nmm).
        T   : (n_tendons,) tendon tensions (N), where (T >= 0) (positive tensions only).
        A   : (3, n_tendons) signed moment-arm matrix (mm).
    """
    def __init__(
            self, 
            pullies, 
            tendon_sign, 
            coupling_ratio : float,
            ) -> None:
        """
        Args:
            pullies : Pulley look-up object.
            tendon_sign : (3, n_tendons) D matrix.
            coupling_ratio  : k such that q_DIP = k*q_PIP.
        """
        self.P = pullies
        self.D = np.asarray(tendon_sign, dtype=float)
        self.k = coupling_ratio

        self.R4 = self._tendon_route_full() 
        self.R = self._tendon_route_generalized(self.R4) 

        self.A = self.D * self.R
        self.internal_id = 3

        if self.D.shape != self.R.shape:
            raise ValueError(f"D shape {self.D.shape} must match R shape {self.R.shape}.")

    def _tendon_route_full(self):
        """
        "Full" (non-generalized) routing matrix (4, n_tendons).
        Rows correspond to joints [splay, MCP, PIP, DIP].
        Columns correspond to tendons [+splay, MCP extensor, DIP extensor, Internal tendon, PIP flexor, MCP flexor, -splay].
        Splay is modeled as two virtual tendons for solving purposes since it is N-config.
        Entries are pulley radii (mm), unsigned.
        """
        R4 = np.array([
                      [self.P.r(0,1), 0,            0,             0,              0,                0, self.P.r(0,1)],
                      [0,             self.P.r(2,1),self.P.r(2,2), 0,              self.P.r(2,3),    self.P.r(2,4), 0],
                      [0,             0,            self.P.r(4,1), self.P.r(4,2),  self.P.r(4,3),    0, 0],
                      [0,             0,            self.P.r(6,1), self.P.r(6,2),  0,                0, 0]], dtype=float)
        return R4
    
    def _tendon_route_generalized(self, R4  : np.ndarray) -> np.ndarray:
        """
        Convert full R4 routing to generalized R3 routing. Combines PIP and DIP rows.

        Returns:
            R3  : (3, ntendons)
        """
        R3 = np.zeros((3, R4.shape[1]), dtype=float)
        R3[0, :] = R4[0, :]
        R3[1, :] = R4[1, :]
        R3[2, :] = R4[2, :] + self.k*R4[3, :]
        return R3
    
    def joint_torques_from_tensions(self, T) -> np.ndarray:
        """ Compute tau = A @ T. """
        T = np.asarray(T, dtype=float).reshape(-1)
        return self.A @ T
    
    def tendon_length_rates_from_qdot(self, qdot) -> np.ndarray:
        """ Tendon length rate from generalized joint rates.

        Returns:
            ldot    : (n_tendons,) tendon length rates (mm/s) for qdot (rads/s). 
        """
        qdot = np.asarray(qdot, dtype=float).reshape(-1)
        return -(self.A.T @ qdot)


    def solve_tensions(self, tau_ref, alpha : float = 0.0):
        """ Solve for (positive) tendon tensions (NNLS) that best achieves the reference joint torques.
        
        Args:
            tau_ref : (3,) reference (command) generalized torques [Splay, MCP, PIPgen] (Nmm).
            alpha   : (>= 0) tension penalty weight [1e-6:1e-3].
        
        Returns:
            T   : (n_tendons,) non-negative tensions (N).
            torque_error_norm   : ||A@T-tau|| (Nmm).
        """
        tau = np.asarray(tau_ref, dtype=float).reshape(-1)
        m = self.A.shape[1]

        if alpha == 0.0:
            T, _ = nnls(self.A, tau)
        else:
            A_aug = np.vstack([self.A, np.sqrt(alpha) * np.eye(m)])
            b_aug = np.concatenate([tau, np.zeros(m)])
            T, _ = nnls(A_aug, b_aug)
        
        torque_err = np.linalg.norm(self.A @ T - tau)
        return T, float(torque_err)