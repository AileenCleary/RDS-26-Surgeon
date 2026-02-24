from __future__ import annotations

from dataclasses import dataclass
import numpy as np
from numpy.typing import NDArray

from rds_finger.model import FingerModel


@dataclass(frozen=True)
class DrumTorque:
    drum: str
    tendon: str
    torque_Nmm: float
    radius_mm: float
    direction: float
    tension_N: float


def compute_drum_torques(
    model: FingerModel,
    tensions: NDArray[np.float64],
) -> list[DrumTorque]:
    """
    Compute torque required at each motor drum given tendon tensions.

    Convention:
      torque_Nmm = direction * radius_mm * tension_N

    - radius is assumed mm
    - tension is N
    - output torque is N*mm
    """
    T = np.asarray(tensions, float).reshape(-1)

    tendon_index = {name: i for i, name in enumerate(model.tendon_order)}

    out: list[DrumTorque] = []
    for dname, dspec in model.drums.items():
        tname = dspec.tendon
        if tname not in tendon_index:
            raise KeyError(f"Drum '{dname}' references tendon '{tname}' not in tendon_order.")
        i = tendon_index[tname]
        tension = float(T[i])

        radius = float(dspec.radius)
        direction = float(dspec.direction)

        torque = direction * radius * tension

        out.append(
            DrumTorque(
                drum=dname,
                tendon=tname,
                torque_Nmm=float(torque),
                radius_mm=radius,
                direction=direction,
                tension_N=tension,
            )
        )

    return out


def motor_torque_summary(drum_torques: list[DrumTorque]) -> dict[str, float]:
    """
    Simple summary: torque per drum.
    If later you model multi-drum motors, aggregate them in config (same motor name).
    """
    return {dt.drum: float(dt.torque_Nmm) for dt in drum_torques}