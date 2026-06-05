from __future__ import annotations
from dataclasses import dataclass
from typing import List, Sequence, Union
import numpy as np # type: ignore

@dataclass(frozen=True)
class TendonEndpoint:
    """Tendon termination/attachment point.
    
    coordinates: <x,y,z> in the global coordinate frame.
    kind: "start"/"end"/"mid"-point of the tendon.
    tendon: Tendon index.
    tendon_name: Name of tendon.
    """
    coordinates: np.ndarray
    kind: str
    tendon: int
    tendon_name: str

@dataclass(frozen=True)
class TendonContact:
    """Tendon contact on a pulley.
    
    pulley_name: Key into pulleys dictionary.
    side: Determines which geometric tangent is used at that pulley in the normal plane of the shaft axis.
    (Hint) The convention I used is in config.py: 
        Point thumb in positive direction of joint axis.
        Imagine pulling on the tendon. Resulting rotation of pulley determines sign,
        i.e., (+CCW,-CW) relative to the positive joint axis."""
    pulley_name: str
    sign: int

TendonElem = Union[TendonEndpoint, TendonContact]

@dataclass(frozen=True)
class TendonPath:
    """Ordered list of elements along a tendon path (proximal to distal).
    
    contacts: List containing TendonEndpoint and TendonContact objects.
    """
    name: str
    contacts: Sequence[TendonElem]