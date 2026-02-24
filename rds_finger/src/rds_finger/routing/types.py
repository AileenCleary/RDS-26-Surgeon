from __future__ import annotations

from dataclasses import dataclass
from typing import Literal, Sequence, Union

import numpy as np

Vec3 = np.ndarray

# Tendon items are either tagged point refs ("DRUM:name", "EP:name")
# or a pulley contact description.
TendonItem = Union[str, "TendonContact"]
TendonItems = Sequence[TendonItem]


@dataclass(frozen=True)
class PulleySpec:
    """Configuration-time pulley definition (local to a shaft + lane)."""
    name: str
    radius: float
    width: float
    shaft: str
    lane: float
    mu: float = 0.0  # optional friction coefficient for capstan model later


@dataclass(frozen=True)
class ShaftSpec:
    """Configuration-time shaft definition attached to an anchor frame."""
    name: str
    diameter: float
    length: float
    axis_local: Vec3
    anchor_frame: str
    center_local: Vec3
    dof_row: int | None
    dof_gain: float = 1.0  # e.g., DIP coupled: theta_shaft = gain * q[dof_row]


@dataclass(frozen=True)
class AnchoredPoint:
    """A point defined in a named frame."""
    name: str
    frame: str
    p_local: Vec3


@dataclass(frozen=True)
class MotorDrumSpec:
    """Motor drum/spool geometry and placement."""
    name: str
    tendon: str
    radius: float
    direction: float         # +1/-1 mapping tendon positive length vs motor angle
    anchor_frame: str
    center_local: Vec3
    axis_local: Vec3


ContactKind = Literal["idler", "fixed"]


@dataclass(frozen=True)
class TendonContact:
    """A tendon contact with a pulley.

    side:
      Routing hint only (+1/-1). It is NOT a winding sign convention.

    kind:
      - "idler": frictionless contact, no spool-length term
      - "fixed": tendon terminates / is fixed on that pulley (adds spool length)
    """
    pulley: str
    side: int
    kind: ContactKind = "idler"
    handoff_90: bool = False
    spool_sign: float = 1.0


@dataclass(frozen=True)
class TendonPathSpec:
    """Ordered tendon path through world objects."""
    name: str
    items: TendonItems


@dataclass(frozen=True)
class BearingSpec:
    """
    Bearing defined as a support on a shaft at a lane offset.

    lane is measured along shaft axis from shaft center (mm),
    consistent with pulley lane convention.
    """
    name: str
    shaft: str
    lane: float
    C: float = 533.786594
    p: float = 3.0
    # later: C0, type, limits, etc.


@dataclass(frozen=True)
class WorldBearing:
    """Resolved bearing in world coordinates (computed from model.world_state)."""
    name: str
    shaft: str
    center: Vec3
    axis: Vec3