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
            raise TypeError(f"{tendon.name}: unsupported element type {type(elem).__name__}")
        p = pulleys[elem.pulley_name]
        tp = p.tangent_points_by_tendon.get(tendon.name)
        if tp is None or len(tp) < 2:
            wrap = p.wrap_angle_by_tendon.get(tendon.name, None)
            raise RuntimeError(""
            )
        return tp[0] if incoming else tp[1]
    
    L = 0.0
    for i in range(len(contacts) - 1):
        a = pt(contacts[i], incoming=False)
        b = pt(contacts[i + 1], incoming=True)
        L += float(np.linalg.norm(b - a))

    for elem in contacts:
        if isinstance(elem, TendonContact):
            p = pulleys[elem.pulley_name]
            L += float(p.radius)*float(p.wrap_angle_by_tendon.get(tendon.name, 0.0))
    
    return float(L)

def tendon_lengths_all(
        tendons: Dict[str, TendonPath],
        pulleys: Dict[str, Pulley],
        tendon_order: List[str],
        *,
        debug: bool = False,
) -> np.ndarray:
    for p in pulleys.values():
        p._clear_runtime()
    calculate_all_wrap_angles(tendons, pulleys)
    Ls = []
    for name in tendon_order:
        L = tendon_length_endpoint_path(tendons[name], pulleys)
        Ls.append(L)

        # if debug:
        #     arcs = []
        #     for elem in tendons[name].contacts:
        #         if isinstance(elem, TendonContact):
        #             p = pulleys[elem.pulley_name]
        #             w = float(p.wrap_angle_by_tendon.get(name, 0.0))
        #             arcs.append((p.name, float(p.radius)*w, w))
        #     arcs.sort(key=lambda x: abs(x[1]), reverse=True)
        #     print(f"[DEBUG] tendon {name}: L={L:.6f} mm, top arcs:")
        #     for (pn, arc, wrap) in arcs[:5]:
        #         print(f"    {pn:12s} arc={arc:12.6f} (wrap={wrap:9.6f} rads)")

    return np.asarray(Ls, dtype=float)