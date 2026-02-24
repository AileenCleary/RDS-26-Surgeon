from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List

import numpy as np

from rds_finger.core.math3d import as3, safe_acos, unit
from rds_finger.model import WorldDrum, WorldEndpoint, WorldPulley
from rds_finger.routing.tangency_parallel import tangents_two_circles_parallel_axis
from rds_finger.routing.tangency_point import tangents_point_circle_parallel_axis
from rds_finger.routing.types import TendonContact, TendonPathSpec

def _append_point(points: List[np.ndarray], p: np.ndarray, eps: float = 1e-9) -> None:
    """Append point unless it is effectively identical to the previous."""
    p = as3(p)
    if not points:
        points.append(p)
        return
    if float(np.linalg.norm(points[-1] - p)) <= eps:
        return
    points.append(p)


def _unit_or_none(v: np.ndarray, eps: float = 1e-12) -> np.ndarray | None:
    v = as3(v)
    n = float(np.linalg.norm(v))
    if n < eps:
        return None
    return (v / n).reshape(3)


def _point_from_item(
    item: str,
    wendpoints: Dict[str, WorldEndpoint],
    wdrums: Dict[str, WorldDrum],
) -> np.ndarray:
    """Resolve a point item like 'EP:name' or 'DRUM:name' to a world point."""
    if not isinstance(item, str):
        raise TypeError(f"Expected str node like 'EP:x' or 'DRUM:y', got {type(item)}")
    kind, name = item.split(":", 1)
    if kind == "EP":
        return as3(wendpoints[name].p)
    if kind == "DRUM":
        return as3(wdrums[name].center)
    raise ValueError(f"Unknown point node tag: {item}")


def _wrap_angle_about_axis(center: np.ndarray, axis: np.ndarray, p_entry: np.ndarray, p_exit: np.ndarray) -> float:
    """Minor-arc wrap angle between entry/exit (in plane normal to axis)."""
    c = as3(center)
    a = unit(as3(axis))
    u = as3(p_entry) - c
    v = as3(p_exit) - c
    u = u - float(np.dot(u, a)) * a
    v = v - float(np.dot(v, a)) * a
    nu = float(np.linalg.norm(u))
    nv = float(np.linalg.norm(v))
    if nu < 1e-9 or nv < 1e-9:
        return 0.0
    u = u / nu
    v = v / nv
    return safe_acos(float(np.dot(u, v)))

def side_preference_cost(
    *,
    center: np.ndarray,
    axis: np.ndarray,
    toward: np.ndarray,
    candidate: np.ndarray,
    side_hint: int,
) -> float:
    """
    Small penalty for picking a tangency on the "wrong side".
    If side_hint==0, disabled.
    """
    if side_hint == 0:
        return 0.0

    c = as3(center)
    a = unit(as3(axis))
    v = as3(toward) - c
    ref = np.cross(a, v)
    if float(np.linalg.norm(ref)) < 1e-12:
        return 0.0
    ref = unit(ref)

    s = float(np.dot(as3(candidate) - c, ref)) * float(side_hint)
    return max(0.0, -s)


def choose_by_side(
    *,
    center: np.ndarray,
    axis: np.ndarray,
    toward: np.ndarray,
    candidates: List[np.ndarray],
    side_hint: int,
) -> np.ndarray:
    """Pick candidate with max signed score according to side hint."""
    if not candidates:
        raise ValueError("No candidates to choose from.")

    if side_hint == 0:
        return as3(candidates[0])

    c = as3(center)
    a = unit(as3(axis))
    v = as3(toward) - c
    ref = np.cross(a, v)
    if float(np.linalg.norm(ref)) < 1e-12:
        ref = np.array([1.0, 0.0, 0.0], dtype=float)
    ref = unit(ref)

    def score(cand: np.ndarray) -> float:
        return float(np.dot(as3(cand) - c, ref)) * float(side_hint)

    return as3(max(candidates, key=score))


def choose_by_continuity_point(
    *,
    prev_point: np.ndarray,
    prev_prev_point: np.ndarray | None,
    candidates: List[np.ndarray],
    center: np.ndarray | None = None,
    axis: np.ndarray | None = None,
    side_hint: int = 0,
    w_cont: float = 1.0,
    w_side: float = 0.05,
) -> np.ndarray:
    """
    Choose candidate point using:
      continuity: 1 - dot(d_prev, d_new)
      + optional side penalty

    If prev_prev_point is None, falls back to side or first.
    """
    if not candidates:
        raise ValueError("No candidates to choose from.")

    prev_point = as3(prev_point)

    if prev_prev_point is None:
        if side_hint == 0 or center is None or axis is None:
            return as3(candidates[0])
        return choose_by_side(center=as3(center), axis=as3(axis), toward=prev_point, candidates=candidates, side_hint=side_hint)

    d_prev = _unit_or_none(prev_point - as3(prev_prev_point))
    if d_prev is None:
        if side_hint == 0 or center is None or axis is None:
            return as3(candidates[0])
        return choose_by_side(center=as3(center), axis=as3(axis), toward=prev_point, candidates=candidates, side_hint=side_hint)

    best: np.ndarray | None = None
    best_cost = float("inf")

    c = as3(center) if center is not None else None
    a = as3(axis) if axis is not None else None

    for cand in candidates:
        cand = as3(cand)
        d_new = _unit_or_none(cand - prev_point)
        if d_new is None:
            continue

        cont = 1.0 - float(np.dot(d_prev, d_new))

        side = 0.0
        if side_hint != 0 and c is not None and a is not None:
            side = side_preference_cost(
                center=c,
                axis=a,
                toward=prev_point,
                candidate=cand,
                side_hint=side_hint,
            )

        cost = w_cont * cont + w_side * side
        if cost < best_cost:
            best = cand
            best_cost = cost

    if best is None:
        if side_hint == 0 or center is None or axis is None:
            return as3(candidates[0])
        return choose_by_side(center=as3(center), axis=as3(axis), toward=prev_point, candidates=candidates, side_hint=side_hint)

    return best

@dataclass
class RoutedPulley:
    """Pulley contact information from a routed tendon (tangency points)."""
    name: str
    entry: np.ndarray
    exit: np.ndarray
    wrap_angle: float
    radius: float
    axis: np.ndarray
    center: np.ndarray


@dataclass
class RoutedTendon:
    """A routed tendon polyline + per-pulley tangency metadata."""
    name: str
    points: List[np.ndarray]
    pulleys: List[RoutedPulley]

def route_tendons(
    tendon: TendonPathSpec,
    wpulleys: Dict[str, WorldPulley],
    wendpoints: Dict[str, WorldEndpoint],
    wdrums: Dict[str, WorldDrum],
) -> RoutedTendon:
    """
    """
    nodes = list(tendon.items)
    if not nodes:
        raise ValueError(f"{tendon.name}: empty tendon path spec")

    if isinstance(nodes[0], TendonContact):
        raise ValueError(f"{tendon.name}: first node cannot be a pulley contact; must start at EP: or DRUM:")

    points: List[np.ndarray] = []
    routed_pulleys: List[RoutedPulley] = []

    prev_point = _point_from_item(nodes[0], wendpoints, wdrums)
    _append_point(points, prev_point)

    entry_is_on_current_pulley = False

    k = 1
    while k < len(nodes):
        cur = nodes[k]

        if not isinstance(cur, TendonContact):
            prev_point = _point_from_item(cur, wendpoints, wdrums)
            _append_point(points, prev_point)
            entry_is_on_current_pulley = False
            k += 1
            continue

        if cur.pulley not in wpulleys:
            raise KeyError(f"{tendon.name}: missing pulley '{cur.pulley}' in wpulleys")

        P = wpulleys[cur.pulley]
        cP = as3(P.center)
        aP = as3(P.axis)
        rP = float(P.radius)

        prev_prev = points[-2] if len(points) >= 2 else None

        if not entry_is_on_current_pulley:
            entry_cands = tangents_point_circle_parallel_axis(prev_point, cP, rP, aP)
            if not entry_cands:
                raise ValueError(f"{tendon.name}: no entry tangents from prev point to pulley {cur.pulley}")

            entry = choose_by_continuity_point(
                prev_point=prev_point,
                prev_prev_point=prev_prev,
                candidates=entry_cands,
                center=cP,
                axis=aP,
                side_hint=int(cur.side),
            )
            _append_point(points, entry)
        else:
            entry = as3(prev_point)

        if k + 1 >= len(nodes):
            if getattr(cur, "kind", "idler") != "fixed":
                raise ValueError(f"{tendon.name}: pulley contact cannot be last unless kind='fixed' (got {getattr(cur,'kind','idler')})")

            routed_pulleys.append(
                RoutedPulley(
                    name=cur.pulley,
                    entry=entry,
                    exit=entry,
                    wrap_angle=0.0,
                    radius=rP,
                    axis=as3(aP),
                    center=as3(cP),
                )
            )
            break

        nxt = nodes[k + 1]

        if isinstance(nxt, TendonContact):
            Q = wpulleys[nxt.pulley]
            cQ = as3(Q.center)
            aQ = as3(Q.axis)
            rQ = float(Q.radius)

            if abs(float(np.dot(unit(aP), unit(aQ)))) < 0.999:
                raise ValueError(f"{tendon.name}: non-parallel consecutive pulleys in v1: {cur.pulley} -> {nxt.pulley}")

            pairs = tangents_two_circles_parallel_axis(cP, rP, cQ, rQ, aP)  
            if not pairs:
                raise ValueError(f"{tendon.name}: no circle-circle tangents for {cur.pulley}->{nxt.pulley}")

            exit_cands = [as3(pP) for (pP, _) in pairs]
            exitP = choose_by_continuity_point(
                prev_point=entry,
                prev_prev_point=(points[-2] if len(points) >= 2 else None),
                candidates=exit_cands,
                center=cP,
                axis=aP,
                side_hint=int(cur.side),
            )

            entryQ: np.ndarray | None = None
            for pP2, pQ2 in pairs:
                if np.allclose(as3(pP2), exitP, atol=1e-9):
                    entryQ = as3(pQ2)
                    break
            if entryQ is None:
                entryQ = min(pairs, key=lambda pr: float(np.linalg.norm(as3(pr[0]) - exitP)))[1]
                entryQ = as3(entryQ)

            _append_point(points, exitP)

            wrap = _wrap_angle_about_axis(cP, aP, entry, exitP)
            routed_pulleys.append(
                RoutedPulley(
                    name=cur.pulley,
                    entry=entry,
                    exit=exitP,
                    wrap_angle=float(wrap),
                    radius=rP,
                    axis=as3(aP),
                    center=as3(cP),
                )
            )

            prev_point = entryQ
            _append_point(points, prev_point)
            entry_is_on_current_pulley = True  
            k += 1
            continue

        next_point = _point_from_item(nxt, wendpoints, wdrums)
        exit_cands = tangents_point_circle_parallel_axis(next_point, cP, rP, aP)
        if not exit_cands:
            raise ValueError(f"{tendon.name}: no exit tangents from pulley {cur.pulley} to next point")

        exitP = choose_by_continuity_point(
            prev_point=entry,
            prev_prev_point=(points[-2] if len(points) >= 2 else None),
            candidates=exit_cands,
            center=cP,
            axis=aP,
            side_hint=int(cur.side),
        )

        _append_point(points, exitP)
        _append_point(points, next_point)

        wrap = _wrap_angle_about_axis(cP, aP, entry, exitP)
        routed_pulleys.append(
            RoutedPulley(
                name=cur.pulley,
                entry=entry,
                exit=exitP,
                wrap_angle=float(wrap),
                radius=rP,
                axis=as3(aP),
                center=as3(cP),
            )
        )

        prev_point = next_point
        entry_is_on_current_pulley = False
        k += 2

    return RoutedTendon(name=tendon.name, points=points, pulleys=routed_pulleys)