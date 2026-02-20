from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Iterable

import numpy as np 

import config as cfg
from components import Pulley, BearingBase, Bearing, Shaft
from shaft_analysis import (
    ShaftAnalysis,
    bearing_equivalent_load,
    L10_revolutions,
    compute_all_pulley_loads_endpoint_path,
    calculate_all_wrap_angles,
)

from utils import _unit

@dataclass(frozen=True)
class Scenario: # change name?
    name: str
    tendon_tensions_N: Dict[str, float]
    mu: float = 0.05
    friction_mode: str = "none"
    load_sf: float = 1.5

@dataclass(frozen=True)
class BearingResult:
    shaft_key: str
    bearing_side: str
    bearing_base_key: str
    Fr_N: float
    Fa_N: float
    P_N: float
    L10_rev: Optional[float]

# def _bearing_choice_from_dict(d: dict) -> BearingChoice:
#     return BearingChoice(
#         part=str(d["part"]),
#         bore_mm=float(d["inner_diameter_mm"]),
#         od_mm=float(d["outer_diameter_mm"]),
#         width_mm=float(d["width_mm"]),
#         C_N=float(d.get("C_N", 0.0)),
#         C0_N=float(d.get("C0_N", 0.0)),
#         X=float(d.get("X", 1.0)),
#         Y=float(d.get("Y", 0.0)),
#         p=float(d.get("p", 3.0)),
#         notes=str(d.get("notes", "")),
#     )

def _fr_fa_from_reaction(
        R_xyz: np.ndarray, 
        shaft_axis: np.ndarray
) -> Tuple[float, float]:
    a = _unit(np.asarray(shaft_axis, float).reshape(3))
    R = np.asarray(R_xyz, float).reshape(3)
    Fa = abs(float(np.dot(R, a)))
    R_rad = R - float(np.dot(R, a)) * a
    Fr = float(np.linalg.norm(R_rad))
    return Fr, Fa

def _set_scenario_pulley_params(sc: Scenario) -> None:
    for p in cfg.ALL_PULLEYS.values():
        p.mu = float(sc.mu)

def _clear_all_pulley_forces() -> None:
    for p in cfg.ALL_PULLEYS.values():
        p._clear_runtime()

def apply_scenario_loads(sc: Scenario) -> None:
    _clear_all_pulley_forces()
    _set_scenario_pulley_params(sc)

    calculate_all_wrap_angles(cfg.TENDONS, cfg.ALL_PULLEYS)

    for tendon_name, tendon in cfg.TENDONS.items():
        T0 = float(sc.tendon_tensions_N.get(tendon_name, 0.0))
        if T0 <= 0.0:
            continue

        touched, _T_end = compute_all_pulley_loads_endpoint_path(
            tendon,
            cfg.ALL_PULLEYS,
            T0,
            friction_mode=sc.friction_mode,
            mu_new=sc.mu,
        )

def pulleys_by_shaft() -> Dict[str, list[Pulley]]:
    shaft_id_to_key: Dict[int, str] = {
        id(shaft): shaft_key for shaft_key, (_bL, _bR, shaft) in cfg.BEARINGS_BY_SHAFT.items()
    }

    out: Dict[str, List[Pulley]] = {k: [] for k in cfg.BEARINGS_BY_SHAFT.keys()}
    for p in cfg.ALL_PULLEYS.values():
        key = shaft_id_to_key.get(id(p.shaft))
        if key is None:
            continue
        out[key].append(p)
    return out


# def tendon_forces_for_scenario(
#         sc: Scenario
# ) -> List[Tuple[str, str, np.ndarray]]:
#     """
#     Returns list of:
#       (tendon_name, pulley_name, force_xyz)
#     """
#     _set_scenario_pulley_params(sc)

#     calculate_all_wrap_angles(cfg.TENDONS, cfg.ALL_PULLEYS)

#     out: List[Tuple[str, str, np.ndarray]] = []

#     for tendon_name, tendon in cfg.TENDONS.items():
#         T0 = float(sc.tendon_tensions_N.get(tendon_name, 0.0))
#         if T0 <= 0.0:
#             continue

#         touched, _T_end = compute_all_pulley_loads_endpoint_path(
#             tendon,
#             cfg.ALL_PULLEYS,
#             T0,
#             friction_mode=sc.friction_mode,
#             mu_new=sc.mu,
#         )

#         for p in touched:
#             F = sc.load_sf * np.asarray(p.force_xyz, float).reshape(3)
#             out.append((tendon_name, p.name, F))

#     return out

# def loads_by_shaft(
#         sc: Scenario,
# ) -> Dict[str, List[Pulley]]:
#     shaft_id_to_key: Dict[int, str] = {
#         id(shaft): shaft_key for shaft_key, (_bL, _bR, shaft) in cfg.BEARINGS_BY_SHAFT.items()
#     }
    
#     shaft_pulleys: Dict[str, List[Pulley]] = {}

#     for _tname, pulley_name, _F in tendon_forces_for_scenario(sc):
#         pulley = cfg.ALL_PULLEYS[pulley_name]
#         key = shaft_id_to_key.get(id(pulley.shaft))
#         if key is None:
#             continue
        
#         if pulley not in shaft_pulleys.setdefault(key, []):
#             shaft_pulleys[key].append(pulley)
#     return shaft_pulleys

def compute_reactions_for_shaft(
        loads: List[Pulley],
        bearing_left_center: np.ndarray,
        bearing_right_center: np.ndarray,
        shaft_axis: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    return ShaftAnalysis.bearing_reactions_3d(
        loads,
        bearing_left_center,
        bearing_right_center,
        shaft_axis,
        use_pulley_s=False
    )[0:2]

def bearing_base_from_instance(
        b: Bearing
) -> BearingBase:
    base = getattr(b, "base", None)
    if base is None:
        raise ValueError("")
    return base

def bearing_base_key_for_instance(
        b: Bearing,
) -> str:
    for k, base in cfg.BEARING_BASES.items():
        if base is b.base: # type: ignore *** why is this an error
            return k
    return "<unknown>"

def evaluate_scenario(
        sc: Scenario, 
) -> Dict[str, List[BearingResult]]:
    apply_scenario_loads(sc)
    
    by_shaft = pulleys_by_shaft()
    out: Dict[str, List[BearingResult]] = {}

    for shaft_key, (bL, bR, shaft) in cfg.BEARINGS_BY_SHAFT.items():
        loads = by_shaft.get(shaft_key, [])
        if not loads:
            continue
        

        scaled_loads: List[Pulley] = [] # ***
        for p in loads:
            if not hasattr(p, "force_xyz"):
                continue
            scaled_loads.append(p)

        R1_xyz, R2_xyz = compute_reactions_for_shaft(
            scaled_loads,
            np.asarray(bL.center, float),
            np.asarray(bR.center, float),
            np.asarray(shaft.axis, float),
        )

        R1_xyz = float(sc.load_sf) * np.asarray(R1_xyz, float).reshape(3)
        R2_xyz = float(sc.load_sf) * np.asarray(R2_xyz, float).reshape(3)

        Fr1, Fa1 = _fr_fa_from_reaction(R1_xyz, shaft.axis)
        Fr2, Fa2 = _fr_fa_from_reaction(R2_xyz, shaft.axis)

        baseL = bearing_base_from_instance(bL)
        baseR = bearing_base_from_instance(bR)
        base_key_L = bearing_base_key_for_instance(bL)
        base_key_R = bearing_base_key_for_instance(bR)

        P1 = bearing_equivalent_load(Fr1, Fa1, baseL, X=baseL.X, Y=baseL.Y)
        P2 = bearing_equivalent_load(Fr2, Fa2, baseR, X=baseR.X, Y=baseR.Y)

        L10_1 = None
        if float(baseL.C) > 0.0 and P1 > 1e-9:
            L10_1 = float(L10_revolutions(baseL.C, P1, float(baseL.p)))
        L10_2 = None
        if float(baseR.C) > 0.0 and P1 > 1e-9:
            L10_2 = float(L10_revolutions(baseR.C, P2, float(baseR.p)))

        out[shaft_key] = [
            BearingResult(shaft_key, "left", base_key_L, Fr1, Fa1, P1, L10_1),
            BearingResult(shaft_key, "right", base_key_R, Fr2, Fa2, P2, L10_2)
        ]

    return out
    
def summarize_results(results: Dict[str, List[BearingResult]]) -> Tuple[float, float]:
    maxP = 0.0
    minL10 = float("inf")
    have_life = False

    for brs in results.values():
        for br in brs:
            maxP = max(maxP, br.P_N)
            if br.L10_rev is not None:
                have_life = True
                minL10 = min(minL10, br.L10_rev)
    
    if not have_life:
        minL10 = float("nan")
    return maxP, minL10

def main():
    scenarios = [
        Scenario(
            name="single_tendon_50N",
            tendon_tensions_N={k: 50.0 for k in cfg.TENDONS.keys()}, #populate with actual
            mu=0.05,
            friction_mode="none",
            load_sf=1.5,
        ),
    ]

    for sc in scenarios:
        res = evaluate_scenario(sc)
        maxP, minL10 = summarize_results(res)

        print(f"\nScenario: {sc.name}")
        print(f"    max P across bearings: {maxP:.3f} N.")
        if not np.isnan(minL10):
            print(" min L10: N/A (bearing base C not set.)")
        else:
            print(f"    min L10: {minL10:.3e} rev")
        
        for shaft_key, brs in res.items():
            for br in brs:
                life = "N/A" if br.L10_rev is None else f"{br.L10_rev:.3e}"
                print(
                    f"  {shaft_key:>4s} {br.bearing_side:<5s} "
                    f"base={br.bearing_base_key:<16s} "
                    f"Fr={br.Fr_N:8.3f} Fa={br.Fa_N:8.3f} "
                    f"P={br.P_N:8.3f} L_10={life}"
                )

if __name__ == "__main__":
    main()