from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from rds_finger.loads.pulley_loads import PulleyLoad
from rds_finger.core.math3d import as3


@dataclass
class ShaftLoad:
    """Net force and moment on a shaft in the world frame."""
    shaft: str
    F_world: np.ndarray
    M_world: np.ndarray

def aggregate_shaft_loads(
    pulley_loads: List[PulleyLoad],
    *,
    pulley_to_shaft: Dict[str, str],
    shaft_centers_world: Dict[str, np.ndarray],
) -> Dict[str, ShaftLoad]:
    """Sum pulley loads onto shafts: net force and net moment about shaft center."""
    F_sum: Dict[str, np.ndarray] = {}
    M_sum: Dict[str, np.ndarray] = {}

    for pl in pulley_loads:
        sname = pulley_to_shaft.get(pl.pulley)
        if sname is None:
            raise KeyError(f"No shaft mapping for pulley {pl.pulley}")
        r0 = shaft_centers_world.get(sname)
        if r0 is None:
            raise KeyError(f"No world center for shaft {sname}")

        r_shaft = as3(r0)
        r_pulley = as3(pl.center)
        F = as3(pl.F_world)
        M = np.cross(r_pulley - r_shaft, F)
        
        if sname not in F_sum:
            F_sum[sname] = np.zeros(3, dtype=float)
            M_sum[sname] = np.zeros(3, dtype=float)

        F_sum[sname] += F
        M_sum[sname] += M

    return {s: ShaftLoad(shaft=s, F_world=F_sum[s], M_world=M_sum[s]) for s in F_sum}