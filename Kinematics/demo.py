from __future__ import annotations

from typing import Any, Dict

import numpy as np  # type: ignore

import config as cfg
from kinematics import RoboticFingerKinematics
from transmission import TendonTransmission
from select_bearings import (
    Scenario,
    loads_by_shaft,
    compute_reactions_for_shaft,
    evaluate_bearing_for_scenario,
    _bearing_choice_from_dict,
    summarize_results,
)


def analyze_tip_force(
    *,
    q: np.ndarray,
    F_tip_xyz_N: np.ndarray,
    mu: float = 0.05,
    friction_mode: str = "decay",
    alpha: float = 1e-6,
    load_sf: float = 1.5,
    verbose: bool = True,
) -> Dict[str, Any]:
    """End-to-end demo:

    tip force -> joint torques -> tendon tensions -> pulley loads -> bearing reactions -> bearing candidates
    """
    q = np.asarray(q, float).reshape(3)
    F_tip_xyz_N = np.asarray(F_tip_xyz_N, float).reshape(3)

    # --- Kinematics (tau in N*mm)
    kin = RoboticFingerKinematics(cfg.LINK_LENGTHS, cfg.FINGERTIP_OFFSET, cfg.COUPLING_RATIO)
    tau_ref = kin.joint_torque_from_tip_force_xyz(q, F_tip_xyz_N)

    tendon_order = cfg.TENDON_ORDER

    dof_count = int(np.asarray(cfg.D_main, float).shape[0])

    trans = TendonTransmission(
        pulleys=cfg.ALL_PULLEYS,
        tendons=cfg.TENDONS,
        tendon_order=cfg.TENDON_ORDER,
        D=np.ones((3, 5), dtype=float),
        dof_count=3,
        coupling_ratio=cfg.COUPLING_RATIO,
        pip_row=2,
        dip_row=3,
        pipgen_row=2,
    )

    print("A=\n", trans.model.A)
    print("tau_ref=", tau_ref)



    T_full, err = trans.solve_tensions(tau_ref, alpha=alpha)

    T_by_name = {name: float(T_full[i]) for i, name in enumerate(tendon_order)}

    if verbose:
        print("\n=== Input ===")
        print(f"q (rad): {q}")
        print(f"F_tip_xyz (N): {F_tip_xyz_N}")

        print("\n=== Joint torques (N*mm) ===")
        print(f"tau_ref: {tau_ref}")

        print("\n=== Solved tendon tensions (N) ===")
        for name in tendon_order:
            print(f"{name:>16s}: {T_by_name[name]:8.3f}")
        print(f"NNLS torque error norm: {err:.6e} (N*mm)")

    scenario_tensions = {tname: float(T_by_name.get(tname, 0.0)) for tname in cfg.TENDONS.keys()}

    sc = Scenario(
        name="tip_force_solved",
        tendon_tensions_N=scenario_tensions,
        mu=mu,
        friction_mode=friction_mode,
        load_sf=load_sf,
    )

    shaft_loads = loads_by_shaft(sc)

    if verbose:
        print("\n=== Shaft loads summary ===")
        for shaft_key, loads in shaft_loads.items():
            total = np.zeros(3)
            for L in loads:
                total += np.asarray(L.force_xyz, float).reshape(3)
            alias = cfg.BEARINGS_BY_SHAFT[shaft_key][2].alias
            print(f"    {shaft_key} (alias='{alias}') : {len(loads)} pulley loads, sumF={total}")

    if verbose:
        print("\n=== Bearing reactions (per shaft) ===")

    for shaft_key, (bL, bR, shaft) in cfg.BEARINGS_BY_SHAFT.items():
        loads = shaft_loads.get(shaft_key, [])
        if not loads:
            continue

        R1, R2 = compute_reactions_for_shaft(
            loads,
            np.asarray(bL.center, float),
            np.asarray(bR.center, float),
            np.asarray(shaft.axis, float),
        )

        if verbose:
            print(f"\n[{shaft_key}] shaft alias='{shaft.alias}'")
            print(f"  Left bearing  reaction R1 (N): {R1}")
            print(f"  Right bearing reaction R2 (N): {R2}")

    candidates_cfg = getattr(cfg, "BEARING_CANDIDATE", [])
    if candidates_cfg:
        candidates = [_bearing_choice_from_dict(d) for d in candidates_cfg]
        if verbose:
            print("\n=== Bearing candidate evaluation ===")
        for c in candidates:
            res = evaluate_bearing_for_scenario(sc, c)
            maxP, minL10 = summarize_results(res)
            life_str = "N/A" if np.isnan(minL10) else f"{minL10:.3e} rev"
            if verbose:
                print(f"  {c.part}: worst P={maxP:.3f} N, worst L10={life_str}")

    return {
        "tau_ref_Nmm": tau_ref,
        "tendon_tensions_full_N": T_by_name,
        "scenario": sc,
        "shaft_loads": shaft_loads,
    }


if __name__ == "__main__":
    q = np.array([0.0, 0.3, 0.5])  # [splay, MCP, PIPgen] rad
    F_tip = np.array([5.0, 0.0, 0.0])  # N in global frame

    analyze_tip_force(
        q=q,
        F_tip_xyz_N=F_tip,
        mu=0.05,
        friction_mode="none",
        alpha=1e-6,
        load_sf=1.5,
        verbose=True,
    )
