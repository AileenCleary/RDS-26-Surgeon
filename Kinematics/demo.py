from __future__ import annotations

from typing import Any, Dict

import numpy as np 

import config as cfg
from kinematics import RoboticFingerKinematics
from transmission import TendonTransmission

from select_bearings import (
    Scenario,
    apply_scenario_loads,
    evaluate_scenario,
    summarize_results,
    compute_reactions_for_shaft,
    pulleys_by_shaft,
)


def analyze_tip_force(
    *,
    q: np.ndarray,
    F_tip_xyz_N: np.ndarray,
    mu: float = 0.05,
    friction_mode: str = "none",
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
    if q.shape[0] != 3:
        raise ValueError("")
    
    tau_ref = np.asarray(kin.joint_torque_from_tip_force_xyz(q, F_tip_xyz_N), float).reshape(-1)

    tendon_order = cfg.TENDON_ORDER
    dof_count = 3

    trans = TendonTransmission(
        cfg=cfg,
        kin=kin,
        pulleys=cfg.ALL_PULLEYS,
        tendons=cfg.TENDONS,
        tendon_order=tendon_order,
        dof_count=dof_count,
        dof_mode="3",
        eps=1e-4,
    )

    D = trans.compute_D(q)

    if verbose:
        print("\n=== Transmission ===")
        print("D = dL/dq shape:", D.shape)
        print(D)
        print("tau_ref (N*mm):", tau_ref)
    
    T_full, err = trans.solve_tensions(tau_ref, alpha=alpha)
    T_full = np.asarray(T_full, float).reshape(-1)
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

    apply_scenario_loads(sc)
    shaft_to_pulleys = pulleys_by_shaft()

    if verbose:
        print("\n=== Bearing Reactions per Shaft ===")
    
    for shaft_key, (bL, bR, shaft) in cfg.BEARINGS_BY_SHAFT.items():
        loads = shaft_to_pulleys.get(shaft_key, [])
        loads_nz = [p for p in loads if float(np.linalg.norm(np.asarray(getattr(p, "force_xyz", 0.0)))) > 1e-12]
        if not loads_nz:
            continue

        R1, R2 = compute_reactions_for_shaft(
            loads_nz,
            np.asarray(bL.center, float),
            np.asarray(bR.center, float),
            np.asarray(shaft.axis, float),
        )

        R1 = load_sf * np.asarray(R1, float).reshape(3)
        R2 = load_sf * np.asarray(R2, float).reshape(3)

        if verbose:
            print(f"\n[{shaft_key}] shaft alias='{shaft.alias}'")
            print(f"    Left bearing reaction R1 (N): {R1}")
            print(f"    Right bearing reaction R2 (N): {R2}")

    results = evaluate_scenario(sc)
    maxP, minL10 = summarize_results(results)
        
    if verbose:
        print("\n=== Bearing base evaluation ===")
        print(f"    worse-case P: {maxP:.3f} N")
        if np.isnan(minL10):
            print(" worse-case L10: N/A (C not set)")
        else:
            print(f"    worst-case L10: {minL10:.3e} rev")

    return {
        "D_dL_dq": D,
        "tau_ref_Nmm": tau_ref,
        "tendon_tensions_N": T_by_name,
        "scenario": sc,
        "bearing_results": results,
    }


if __name__ == "__main__":
    q = np.array([0.0, 0.3, 0.5])  # [splay, MCP, PIPgen] rad
    F_tip = np.array([20.0, 0.0, 0.0])  # N in global frame

    analyze_tip_force(
        q=q,
        F_tip_xyz_N=F_tip,
        mu=0.05,
        friction_mode="none",
        alpha=1e-6,
        load_sf=1.5,
        verbose=True,
    )
