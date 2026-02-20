from __future__ import annotations

from typing import Dict, Literal

import numpy as np 

from components import Pulley, Shaft
from kinematics import RoboticFingerKinematics

# does joint 0 really move

def _unit3(v: np.ndarray) -> np.ndarray:
    v = _as3(v)
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("")
    return v / n

def _as3(x) -> np.ndarray:
    return np.asarray(x, dtype=float).reshape(3)

def update_config_geometry(
        *,
        cfg,
        kin: RoboticFingerKinematics,
        q: np.ndarray,
        dof_mode: Literal["3", "4"] = "3",
) -> None:
    # print("[DBG] update_config_geometry q=", np.asarray(q, float).reshape(-1))
    # print("[DBG] before O1=", getattr(cfg, "joint_1").center)
    origins = kin.frame_origins(q, dof_mode=dof_mode)

    cfg.joint_0.center = _as3(origins["O0"])
    cfg.joint_1.center = _as3(origins["O1"])
    cfg.joint_2.center = _as3(origins["O2"])
    cfg.joint_3.center = _as3(origins["O3"])

    cfg.joint_A.center = 0.5*(_as3(origins["O0"]) + _as3(origins["O1"]))
    cfg.joint_B.center = 0.5*(_as3(origins["O1"]) + _as3(origins["O2"]))
    cfg.joint_C.center = 0.5*(_as3(origins["O2"]) + _as3(origins["O3"]))
    # print("[DBG] after  O1=", cfg.joint_1.center)
    for p in cfg.ALL_PULLEYS.values():
        p.axis = _unit3(p.shaft.axis) # ***
        p.center = _as3(p.shaft.center) + float(p.lane)*p.axis