from __future__ import annotations

from typing import Dict

import numpy as np

from rds_finger.core.frames import Pose
from rds_finger.core.math3d import rot_axis_angle, as3
from rds_finger.kinematics.coupling import dip_from_pip

def compute_frames(
    q: np.ndarray,
    *,
    link_lengths: Dict[str, float],
    coupling_ratio: float = 0.7,
) -> Dict[str, Pose]:
    """Compute world poses of frames O0 to O3 for the 3 DOF link chain.

    q:  Joint vector [splay, mcp, pip] (rads).
    link_lengths:   Dictionary mapping link names (phalanx name) to their length.
    coupling_ratio: Coupling ratio relating thetaDIP to thetaPIP.
    """
    q = as3(q)
    th_splay, th_mcp, th_pip = map(float, q)
    th_dip = dip_from_pip(th_pip, coupling_ratio)
    
    R0 = rot_axis_angle([0,0,1], th_splay)
    O0 = Pose(R0, np.zeros(3))

    Rm = rot_axis_angle([0,1,0], th_mcp)
    O1 = O0 @ Pose(Rm, np.array([link_lengths["splay"], 0, 0], dtype=float))

    Rp = rot_axis_angle([0,1,0], th_pip)
    O2 = O1 @ Pose(Rp, np.array([link_lengths["proximal"], 0, 0], dtype=float))

    Rd = rot_axis_angle([0,1,0], th_dip)
    O3 = O2 @ Pose(Rd, np.array([link_lengths["middle"], 0, 0], dtype=float))

    return {"O0": O0, "O1": O1, "O2": O2, "O3": O3}