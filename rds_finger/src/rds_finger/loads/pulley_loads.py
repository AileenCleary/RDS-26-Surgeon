from __future__ import annotations

from dataclasses import dataclass
from typing import List

import numpy as np

from rds_finger.core.math3d import unit, as3
from rds_finger.routing.router import RoutedTendon


@dataclass
class PulleyLoad:
    """Net load applied to a pulley by a single tendon in the world frame."""
    pulley: str
    tendon: str
    tension: float
    F_world: np.ndarray
    entry: np.ndarray
    exit: np.ndarray
    center: np.ndarray
    axis: np.ndarray

def _find_point_index(
        pts: List[np.ndarray], 
        p: np.ndarray, 
        atol: float = 1e-7
) -> int:
    """Return index of the closest point in [pts] to point p.
    
    Used for matching tangent points to route points.
    """
    p = as3(p)
    best_i = -1
    best_d = np.inf

    for i, q in enumerate(pts):
        d = float(np.linalg.norm(as3(q) - p))
        if d < best_d:
            best_d = d
            best_i = i

    if not np.isfinite(best_d) or best_d > atol:
        raise ValueError(f"Could not match tangency point to polyline (min dist {best_d}).")
    return best_i

def _dir_prev_or_next(
        pts: List[np.ndarray], 
        i: int,
) -> np.ndarray:
    """Unit direction away from point at index i along incident segment.
    
    Preference for prev. neighbor if exists, else next neighbor.
    """
    p = as3(pts[i])
    if i > 0:
        return unit(as3(pts[i - 1]) - p)
    if i + 1 < len(pts):
        return unit(as3(pts[i + 1]) - p)
    raise ValueError("")

def _dir_next_or_prev(
        pts: List[np.ndarray], 
        i: int,
) -> np.ndarray:
    """Unit direction away from point at index i along incident segment.
    
    Preference for next neighbor if exists, else prev. neighbor.
    """
    p = as3(pts[i])
    if i + 1 < len(pts):
        return unit(as3(pts[i + 1]) - p)
    if i > 0:
        return unit(as3(pts[i - 1]) - p)
    raise ValueError("")

def pulley_loads_frictionless(
    rt: RoutedTendon,
    tendon_name: str,
    tension: float,
    atol_match: float = 1e-7,
) -> List[PulleyLoad]:
    """Compute per-pulley net load vectors in frictionless case.

    Two-sided (pass-over) pulley: F = T * t_in + T * t_out
    One-sided (termination or polyline endpoint on the pulley): F = T * t

    Where each t is a unit direction along a tendon segment pointing away from the pulley tangency.
    """
    T = float(tension)
    if T < 0.0:
        raise ValueError("Tension must be nonnegative.")

    pts = rt.points
    loads: List[PulleyLoad] = []

    for rp in rt.pulleys:
        if rp.entry is None or rp.exit is None:
            continue

        entry = as3(rp.entry)
        exit = as3(rp.exit)

        i_entry = _find_point_index(pts, entry, atol=atol_match)
        i_exit = _find_point_index(pts, exit, atol=atol_match)

        if i_entry == i_exit:
            dirs: List[np.ndarray] = []
            p = as3(pts[i_entry])

            if i_entry > 0:
                dirs.append(unit(as3(pts[i_entry - 1]) - p))
            if i_entry + 1 < len(pts):
                dirs.append(unit(as3(pts[i_entry + 1]) - p))

            if len(dirs) == 0:
                raise ValueError("")
            
            F = T * dirs[0] if len(dirs) == 1 else T * (dirs[0] + dirs[1])
        else:
            d_in = _dir_prev_or_next(pts, i_entry)
            d_out = _dir_next_or_prev(pts, i_exit)
            F = T * (d_in + d_out)

        loads.append(
            PulleyLoad(
                pulley=rp.name,
                tendon=tendon_name,
                tension=T,
                F_world=as3(F),
                entry=entry,
                exit=exit,
                center=as3(rp.center),
                axis=as3(rp.axis),
            )
        )

    return loads