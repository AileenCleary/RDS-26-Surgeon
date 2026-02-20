from __future__ import annotations

from typing import Dict, List

import numpy as np

from components import Pulley
from tendon_types import TendonContact, TendonEndpoint, TendonPath
from shaft_analysis import calculate_all_wrap_angles

def tendon_length_endpoint_path(
        tendon: TendonPath,
        pulleys: Dict[str, Pulley],
) -> float:
    contacts = list(tendon.contacts)
    if len(contacts) < 2:
        return 0.0
    
    def pt(
            elem,
            *,
            incoming: bool,
    ) -> np.ndarray:
        if isinstance(elem, TendonEndpoint):
            return np.asarray(elem.coordinates, float).reshape(3)
        if not isinstance(elem, TendonContact):
            raise TypeError("")
        p = pulleys[elem.pulley_name]
        if len(p.tangent_points) < 2:
            raise RuntimeError("")
        return p.tangent_points[0] if incoming else p.tangent_points[1]
    
    L = 0.0
    for i in range(len(contacts) - 1):
        a = pt(contacts[i], incoming=False)
        b = pt(contacts[i + 1], incoming=True)
        L += float(np.linalg.norm(b - a))

    for elem in contacts:
        if isinstance(elem, TendonContact):
            p = pulleys[elem.pulley_name]
            L += float(p.radius)*float(p.wrap_angle)
    
    return float(L)

def tendon_lengths_all(
        tendons: Dict[str, TendonPath],
        pulleys: Dict[str, Pulley],
        tendon_order: List[str],
) -> np.ndarray:
    calculate_all_wrap_angles(tendons, pulleys)
    return np.array(
        [tendon_length_endpoint_path(tendons[name], pulleys) for name in tendon_order],
        dtype=float
    )