from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np # type: ignore

import config as cfg
from shaft_analysis import (
    PulleyLoad,
    ShaftAnalysis,
    compute_tendon_pulley_loads,
    bearing_equivalent_load,
    L10_revolutions,
    _unit,
)

@dataclass(frozen=True)
class Scenario: # change name?
    name: str
    tendon_tensions_N: Dict[str, float]
    mu: float = 0.05
    friction_mode: str = "decay"
    use_tan_wrap: bool = True
    use_tan_force: bool = True
    load_sf: float = 1.5

@dataclass(frozen=True)
class BearingChoice:
    part: str
    bore_mm: float
    od_mm: float
    width_mm: float
    C_N: float
    C0_N: float
    X: float
    Y: float
    p: float
    notes: str = ""

@dataclass(frozen=True)
class BearingResult:
    shaft_key: str
    bearing_side: str
    Fr_N: float
    Fa_N: float
    P_N: float
    L10_rev: Optional[float]

def _bearing_choice_from_dict(d: dict) -> BearingChoice:
    return BearingChoice(
        part=str(d["part"]),
        bore_mm=float(d["inner_diameter_mm"]),
        od_mm=float(d["outer_diameter_mm"]),
        width_mm=float(d["width_mm"]),
        C_N=float(d.get("C_N", 0.0)),
        C0_N=float(d.get("C0_N", 0.0)),
        X=float(d.get("X", 1.0)),
        Y=float(d.get("Y", 0.0)),
        p=float(d.get("p", 3.0)),
        notes=str(d.get("notes", "")),
    )

def _fr_fa_from_reaction(R_xyz: np.ndarray, shaft_axis: np.ndarray) -> Tuple[float, float]:
    a = _unit(np.asarray(shaft_axis, float).reshape(3))
    R = np.asarray(R_xyz, float).reshape(3)
    Fa = abs(float(np.dot(R, a)))
    R_rad = R - float(np.dot(R, a)) * a
    Fr = float(np.lingalg.norm(R_rad))
    return Fr, Fa

def tendon_forces_for_scenario(sc: Scenario) -> List[Tuple[str, str, np.ndarray]]:
    out: List[Tuple[str, str, np.ndarray]] = []
    for tendon_name, tendon in cfg.TENDONS.items():
        T0 = float(sc.tendon_tensions_N.get(tendon_name, 0.0))
        if T0 <= 0.0:
            continue

        for p in cfg.ALL_PULLEYS.values():
            p.mu = sc.mu

        forces, _T_end = compute_tendon_pulley_loads(
            tendon,
            cfg.ALL_PULLEYS,
            T0,
            friction_mode=sc.friction_mode,
            mu_new=sc.mu,
            use_tan_wrap=sc.use_tan_wrap,
            use_tan_force=sc.use_tan_force,
        )

        for pcur, F in forces:
            out.append((tendon_name, pcur.name, sc.load_sf * np.asarray(F, float).reshape(3)))

    return out

def loads_by_shaft(sc: Scenario) -> Dict[object, List[PulleyLoad]]:
    shaft_loads: Dict[object, List[PulleyLoad]] = {}

    for _tname, pulley_name, F in tendon_forces_for_scenario(sc):
        pulley = cfg.ALL_PULLEYS[pulley_name]
        sh = pulley.shaft
        shaft_loads.setdefault(sh, []).append(
            PulleyLoad(pulley=pulley, force_xyz=F, s_mm=0.0)
        )
    
    return shaft_loads

def compute_reactions_for_shaft(
        loads: List[PulleyLoad],
        bearing_left_center: np.ndarray,
        bearing_right_center: np.ndarray,
        shaft_axis: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    R1_xyz, R2_xyz, _loads_out, _L = ShaftAnalysis.bearing_reactions_3d(
        loads,
        bearing_left_center,
        bearing_right_center,
        shaft_axis,
    )
    return R1_xyz, R2_xyz

def evaluate_bearing_for_scenario(sc: Scenario, choice: BearingChoice) -> Dict[str, List[BearingResult]]:
    out: Dict[str, List[BearingResult]] = {}
    shaft_loads = loads_by_shaft(sc)

    for shaft_key, (bL, bR, shaft) in cfg.BEARINGS_BY_SHAFT.items():
        loads = shaft_loads.get(shaft, [])
        if not loads:
            continue

        R1_xyz, R2_xyz = compute_reactions_for_shaft(
            loads,
            np.asarray(bL.center, float),
            np.asarray(bR.center, float),
            np.asarray(shaft.axis, float),
        )

        Fr1, Fa1 = _fr_fa_from_reaction(R1_xyz, shaft.axis)
        Fr2, Fa2 = _fr_fa_from_reaction(R2_xyz, shaft.axis)

        P1 = choice.X*Fr1 + choice.Y*Fa1 #bearing equivalent load
        P2 = choice.X*Fr2 + choice.Y*Fa2

        L10_1 = None
        L10_2 = None
        if choice.C_N > 0.0 and P1 > 1e-9 and P2 > 1e-9:
            L10_1 = float(L10_revolutions(choice.C_N, P1, choice.p))
            L10_2 = float(L10_revolutions(choice.C_N, P2, choice.p))

        out[shaft_key] = [
            BearingResult(shaft_key, "left", Fr1, Fa1, P1, L10_1),
            BearingResult(shaft_key, "right", Fr2, Fa2, P2, L10_2)
        ]

        return out
    
    def passes_geometry(choice: BearingChoice) -> bool:
        return (
            choice.bore_mm >= cfg.MIN_BORE_MM
            # and any other constraints..
        )
    
    def summarize_results(results: Dict[str, List[BearingResult]]) -> Tuple[float, float]:
        maxP = 0.0
        minL10 = float("inf")
        have_life = False

        for _shaft_key, brs in results.items():
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
                friction_mode="decay",
                use_tan_wrap=True,
                use_tan_force=True,
                load_sf=1.5,
            ),
        ]

        candidates = [_bearing_choice_from_dict(d) for d in cfg.BEARING_CANDIDATE]
        candidates = [c for c in candidates if passes_geometry(c)]

        for c in candidates:
            worse_maxP = 0.0
            worst_minL10 = float("inf")
            any_life = False

            for sc in scenarios:
                res = evaluate_bearing_for_scenario(sc, c)
                maxP, minL10 = summarize_results(res)
                worst_maxP = max(worse_maxP, maxP)

                if not np.isnan(minL10):
                    any_life = True
                    worst_minL10 = min(worst_minL10, minL10)

            life_str = f"{worst_minL10: .3e}rev" if any_life else "N/A (C not set)"
            print(f"\nCandidate: {c.part}")
            print(f"    worst-case P: {worst_maxP: .2f}")
            print(f"    worst-case L10: {life_str}")
