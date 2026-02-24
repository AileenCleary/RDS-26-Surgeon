from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel
from rds_finger.loads.loads import ShaftLoad
from typing import Dict


@dataclass(frozen=True)
class ShaftStress:
    shaft: str
    diameter_mm: float
    M_Nmm: float
    sigma_b_MPa: float
    von_mises_MPa: float


def _sigma_bending_MPa(M_Nmm: float, d_mm: float) -> float:
    # sigma = 32 M / (pi d^3), units N/mm^2 = MPa
    d = float(d_mm)
    if d <= 0.0:
        raise ValueError("shaft diameter must be positive")
    return float(32.0 * float(M_Nmm) / (np.pi * d**3))


def compute_shaft_stresses(
    model: FingerModel,
    shaft_loads: Dict[str, ShaftLoad],
) -> list[ShaftStress]:
    """
    Conservative bending-only stress summary from ShaftLoad.M_world.

    - M := ||M_world||
    - sigma_b := 32 M / (pi d^3)
    - von_mises := |sigma_b| (torsion not modeled yet)

    If you later add torsion, update von Mises to sqrt(sigma^2 + 3 tau^2).
    """
    out: list[ShaftStress] = []
    for sl in shaft_loads.values():
        if sl.shaft not in model.shafts:
            raise KeyError(f"ShaftLoad references unknown shaft '{sl.shaft}'.")
        d = float(model.shafts[sl.shaft].diameter)
        M = float(np.linalg.norm(np.asarray(sl.M_world, float).reshape(3)))
        sigma = _sigma_bending_MPa(M, d)
        out.append(
            ShaftStress(
                shaft=sl.shaft,
                diameter_mm=d,
                M_Nmm=M,
                sigma_b_MPa=sigma,
                von_mises_MPa=abs(sigma),
            )
        )
    return out


def worst_shaft_stress(stresses: list[ShaftStress]) -> ShaftStress | None:
    if not stresses:
        return None
    return max(stresses, key=lambda x: x.von_mises_MPa)