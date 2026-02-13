import numpy as np # pyright: ignore[reportMissingImports]

from kinematics import RoboticFingerKinematics
from transmission import TendonTransmission
from shaft_analysis import ShaftAnalysis

class RoboticFinger:
    """
    High-level structure composed of:
        - RoboticFingerKinematics: Jacobian/FK, static transforms
        - TendonTransmission: Tendon routing, joint mapping, moment arm
        - ShaftAnalysis: Pulley/shaft/bearing load utilities.

    State Parameters:
        q   : Generalized joint angles [Splay, MCP, PIP] (rads).
        tip_pos, tip_R  : Fingertip pose computed via kinematics.
        T   : Last solved tendon tensions (N) (>=0).
        tau : Last joint torque target (Nmm) for generalized joints.
        err_norm   : Last torque error norm from solving tension.

    Units:
        - (mm, N, Nmm, rads)
    """
    def __init__(
            self, 
            link_lengths, 
            joint_angles, 
            motor, 
            sf: float, 
            pulleys, 
            tendon_sign, 
            fingertip_pos: float,
        ) -> None:

        self.motor = motor
        self.sf = sf
        self.pulleys = pulleys

        # Coupling ratio.
        self.k = float(self.pulleys.r(4,2) / self.pulleys.r(6,2))

        # Sub-systems.
        self.kin = RoboticFingerKinematics(link_lengths, fingertip_pos, self.k)
        self.trans = TendonTransmission(pulleys, tendon_sign, self.k)
        self.shaft = ShaftAnalysis()

        # State/last outputs.
        self._q = np.asarray(joint_angles, dtype=float).reshape(3)
        self.tip_pos : np.ndarray | None = None
        self.tip_R : np.ndarray | None = None
        self.T : np.ndarray | None = None
        self.tau : np.ndarray | None = None
        self.err_norm: float | None = None

        self.update_kinematics()

    @property
    def q(self) -> np.ndarray:
        """ Generalized joint angles [Splay, MCP, PIP] (rad). """
        return self._q.copy()
    
    @q.setter
    def q(self, q_new) -> None:
        self._q = np.asarray(q_new, dtype=float).reshape(3)
        self.update_kinematics()

    def update_kinematics(self):
        """ Recompute fingertip pose parameters from current q. """
        self.tip_pos, self.tip_R = self.kin.tip_pose(self._q)



    def tensions_from_joint_torque(
            self, 
            tau_ref, 
            preload : float = 0.0,
            *,
            alpha : float = 1e-3,
            beta_preload : float = 1e-3,
            ) -> np.ndarray:
        # self.tau = np.asarray(tau_ref, dtype=float).reshape(3)
        # self.T = self.trans.solve_tensions_qp(self.tau, preload)
        # return self.T.copy()
        self.tau = np.asarray(tau_ref, dtype=float).reshape(-1)
        T, rnorm = self.trans.solve_tensions_reg_nnls(self.tau, 1e-3, preload)
        self.T = T
        self.rnorm = rnorm
        return self.T.copy()

    def tensions_from_tip_force_xy(self, F_xy, preload=0.0):
        tau_ref = self.kin.joint_torque_from_tip_force_xy(self._q, F_xy)
        return self.tensions_from_joint_torque(tau_ref, preload)
    
    def joint_torques_from_tensions(self, T):
        return self.trans.joint_torques_from_tensions(T)