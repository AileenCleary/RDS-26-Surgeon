from __future__ import annotations

from dataclasses import dataclass
from typing import List, Dict

import numpy as np

from rds_finger.model import FingerModel
from rds_finger.statics.bearing_reactions import solve_two_bearing_reactions
from rds_finger.loads.loads import compute_all_pulley_and_shaft_loads  


@dataclass(frozen=True, slots=True)
class BearingLoad:
    """Reaction load at bearing expressed in the world frame."""
    bearing: str
    shaft: str
    R_world: np.ndarray


def compute_all_bearing_loads(
    model: FingerModel,
    q: np.ndarray,
    tensions: np.ndarray,
) -> List[BearingLoad]:
    """Compute bearing reaction loads across all shafts."""
    q = np.asarray(q, dtype=float).reshape(-1)
    _, shaft_loads = compute_all_pulley_and_shaft_loads(model, q, tensions)

    frames, _, wpulleys, wendpoints, wdrums, wshafts, wbearing = model.world_state(q)  

    bearings_by_shaft: Dict[str, List[str]] = {}
    for bname, b in wbearing.items():
        bearings_by_shaft.setdefault(b.shaft, []).append(bname)

    out: List[BearingLoad] = []
    for shaft_name, sl in shaft_loads.items():
        bearing_names = bearings_by_shaft.get(shaft_name)
        if not bearing_names:
            continue

        b1 = wbearing[bearing_names[0]]
        b2 = wbearing[bearing_names[1]]
        
        shaft_center = np.asarray(wshafts[shaft_name].axis.p, dtype=float).reshape(3)

        R1, R2 = solve_two_bearing_reactions(
            F_world=np.asarray(sl.F_world, dtype=float).reshape(3),
            M_world=np.asarray(sl.M_world, dtype=float).reshape(3),
            b1_center=np.asarray(b1.center, dtype=float).reshape(3),
            b2_center=np.asarray(b2.center, dtype=float).reshape(3),
            shaft_center=np.asarray(shaft_center, dtype=float).reshape(3),
        )
        out.append(BearingLoad(bearing=b1.name, shaft=shaft_name, R_world=np.asarray(R1, dtype=float).reshape(3)))
        out.append(BearingLoad(bearing=b2.name, shaft=shaft_name, R_world=np.asarray(R2, dtype=float).reshape(3)))

    return out