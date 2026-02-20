from __future__ import annotations

from typing import Dict, Literal

import numpy as np 

from components import Pulley, Shaft
from kinematics import RoboticFingerKinematics

# does joint 0 really move

def _as3(x) -> np.ndarray:
    return np.asarray(x, dtype=float).reshape(3)

def update_config_geometry(
        *,
        cfg,
        kin: RoboticFingerKinematics,
        q: np.ndarray,
        dof_mode: Literal["3", "4"] = "3",
) -> None:
    origins = kin.frame_origins(q, dof_mode=dof_mode)

    cfg.joint_0.center = _as3(origins["O0"])
    cfg.joint_1.center = _as3(origins["O1"])
    cfg.joint_2.center = _as3(origins["O2"])
    cfg.joint_3.center = _as3(origins["O3"])

    cfg.joint_A.center = 0.5*(_as3(origins["O0"]) + _as3(origins["O1"]))
    cfg.joint_B.center = 0.5*(_as3(origins["O1"]) + _as3(origins["O2"]))
    cfg.joint_C.center = 0.5*(_as3(origins["O2"]) + _as3(origins["O3"]))

    cfg.joint_Z.center = cfg.joint_Z.center

    for p in cfg.ALL_PULLEYS.values():
        p.axis = _as3(p.axis) / float(np.linalg.norm(_as3(p.axis)))
        p.center = _as3(p.shaft.center) + float(p.lane)*_as3(p.axis)