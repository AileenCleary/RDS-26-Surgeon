from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel
from rds_finger.loads.bearing_loads import BearingLoad


@dataclass(frozen=True)
class BearingLife:
    bearing: str
    shaft: str
    P_N: float 
    C_N: float
    p: float
    L10_rev: float


def L10_revs(C_N: float, P_N: float, p: float) -> float:
    """Basic L10 bearing life model (ISO 281 form):

    L10 (million rev) = (C/P)^p
    L10_rev = 1e6 * (C/P)^p
    """
    C = float(C_N)
    P = float(P_N)
    if P <= 0.0:
        return float("inf")
    return float(1e6 * (C / P) ** float(p))


def compute_bearing_lives(
    model: FingerModel,
    bearing_loads: list[BearingLoad],
) -> list[BearingLife]:
    """
    Convert BearingLoad -> BearingLife using model.bearings ratings.

    Equivalent load:
      P := ||R_world||   (simple conservative-ish scalarization)
    """
    out: list[BearingLife] = []
    for bl in bearing_loads:
        if bl.bearing not in model.bearings:
            raise KeyError(f"Missing BearingSpec for bearing '{bl.bearing}'. Add it in config/model.")
        spec = model.bearings[bl.bearing]
        R = np.asarray(bl.R_world, float).reshape(3)
        P = float(np.linalg.norm(R))
        L10 = L10_revs(spec.C, P, spec.p)
        out.append(
            BearingLife(
                bearing=bl.bearing,
                shaft=bl.shaft,
                P_N=P,
                C_N=float(spec.C),
                p=float(spec.p),
                L10_rev=L10,
            )
        )
    return out


def worst_bearing_life(bearing_lives: list[BearingLife]) -> BearingLife | None:
    if not bearing_lives:
        return None
    return min(bearing_lives, key=lambda x: x.L10_rev)