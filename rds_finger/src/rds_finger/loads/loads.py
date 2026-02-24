from __future__ import annotations

from typing import Tuple, List, Dict

import numpy as np

from rds_finger.model import FingerModel
from rds_finger.routing.router import route_tendons
from rds_finger.loads.pulley_loads import pulley_loads_frictionless, PulleyLoad
from rds_finger.loads.shaft_loads import aggregate_shaft_loads, ShaftLoad


def compute_all_pulley_and_shaft_loads(
    model: FingerModel,
    q: np.ndarray,
    tensions: np.ndarray,
) -> Tuple[List[PulleyLoad], Dict[str, ShaftLoad]]:
    """Route all tendons at q, compute idler pulley loads from tensions, and aggregate shaft loads."""
    q = np.asarray(q, dtype=float).reshape(-1)
    T = np.asarray(tensions, dtype=float).reshape(-1)
    
    n_tendons = len(model.tendon_order)
    if T.size != n_tendons:
        raise ValueError("")
    
    frames, _, wpulleys, wendpoints, wdrums, *_ = model.world_state(q)

    pulley_to_shaft: Dict[str, str] = {p.name: p.shaft for p in model.pulleys.values()}

    shaft_centers_world: Dict[str, np.ndarray] = {}
    for sname, sspec in model.shafts.items():
        F = frames[sspec.anchor_frame]
        shaft_centers_world[sname] = np.asarray(F.apply(sspec.center_local), dtype=float).reshape(3)

    pulley_loads: list[PulleyLoad] = []

    for i, tname in enumerate(model.tendon_order):
        spec = model.tendons[tname]
        rt = route_tendons(spec, wpulleys, wendpoints, wdrums)
        pulley_loads.extend(
            pulley_loads_frictionless(
                rt, 
                tendon_name=tname, 
                tension=float(T[i])
            )
        )

    shaft_loads = aggregate_shaft_loads(
        pulley_loads, 
        pulley_to_shaft=pulley_to_shaft, 
        shaft_centers_world=shaft_centers_world,
    )
    return pulley_loads, shaft_loads