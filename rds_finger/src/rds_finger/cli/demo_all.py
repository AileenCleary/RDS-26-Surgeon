from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterator

import numpy as np

from rds_finger.model import build_model
from rds_finger.analysis.pipeline import analyze_tip_force
from rds_finger.analysis.loads.bearing_life import worst_bearing_life
from rds_finger.analysis.loads.shaft_stress import worst_shaft_stress

def _fmt_vec3(v: Any, unit: str = "") -> str:
    a = np.asarray(v, float).reshape(-1)
    if a.size != 3:
        return str(a)
    if unit:
        return f"[{a[0]: .3f}, {a[1]: .3f}, {a[2]: .3f}] {unit}"
    return f"[{a[0]: .3f}, {a[1]: .3f}, {a[2]: .3f}]"


def _fmt_mat(A: Any) -> str:
    return np.array2string(np.asarray(A, float), precision=6, suppress_small=True)


def _as_vec3(v: Any) -> np.ndarray:
    a = np.asarray(v, float).reshape(-1)
    if a.size != 3:
        raise ValueError(f"Expected 3-vector, got shape {a.shape}.")
    return a.reshape(3)


# ------------------------------- load adapters -------------------------------

@dataclass(frozen=True)
class _LoadVec:
    """Normalized representation of any load item with a single vector field."""
    name: str
    vec: np.ndarray


@dataclass(frozen=True)
class _ShaftLoadVec:
    """Normalized representation of any shaft load item with force + moment."""
    shaft: str
    F: np.ndarray
    M: np.ndarray


def _iter_values(x: Any) -> Iterator[Any]:
    """
    Iterate through values for a container that might be:
      - list/tuple of items
      - dict[name -> item] or dict[name -> raw_vector]
      - single item
    """
    if x is None:
        return
        yield  # pragma: no cover
    if isinstance(x, dict):
        for v in x.values():
            yield v
        return
    if isinstance(x, (list, tuple)):
        for v in x:
            yield v
        return
    yield x


def _normalize_pulley_loads(pulley_loads: Any) -> list[_LoadVec]:
    """
    Accepts multiple shapes:
      - list[PulleyLoad] where PulleyLoad has .pulley and .F_world
      - dict[pulley -> vec3]
      - list[dict] with keys {"pulley", "F_world"} or {"name","F_world"}
      - list[(name, vec3)]
    """
    out: list[_LoadVec] = []
    if isinstance(pulley_loads, dict):
        for k, v in pulley_loads.items():
            try:
                out.append(_LoadVec(str(k), _as_vec3(v)))
            except Exception:
                continue
        return out

    for item in _iter_values(pulley_loads):
        if hasattr(item, "F_world"):
            name = str(getattr(item, "pulley", getattr(item, "name", "pulley")))
            out.append(_LoadVec(name, _as_vec3(getattr(item, "F_world"))))
            continue

        if isinstance(item, dict):
            if "F_world" not in item:
                continue
            name = str(item.get("pulley", item.get("name", "pulley")))
            out.append(_LoadVec(name, _as_vec3(item["F_world"])))
            continue

        if isinstance(item, (tuple, list)) and len(item) >= 2:
            name = str(item[0])
            try:
                out.append(_LoadVec(name, _as_vec3(item[1])))
            except Exception:
                continue

    return out


def _normalize_shaft_loads(shaft_loads: Any) -> list[_ShaftLoadVec]:
    """
    Accepts multiple shapes:
      - list[ShaftLoad] where ShaftLoad has .shaft, .F_world, .M_world
      - dict[shaft -> ShaftLoad] or dict[shaft -> {"F_world":..., "M_world":...}]
      - list[dict] with keys {"shaft","F_world","M_world"} (or {"F","M"})
      - list[(F, M)] (name-less) is allowed but will be labeled "shaft"
    """
    out: list[_ShaftLoadVec] = []

    if isinstance(shaft_loads, dict):
        for k, v in shaft_loads.items():
            shaft = str(k)
            if hasattr(v, "F_world") and hasattr(v, "M_world"):
                out.append(_ShaftLoadVec(shaft, _as_vec3(v.F_world), _as_vec3(v.M_world)))
                continue
            if isinstance(v, dict):
                F = v.get("F_world", v.get("F", None))
                M = v.get("M_world", v.get("M", None))
                if F is None or M is None:
                    continue
                out.append(_ShaftLoadVec(shaft, _as_vec3(F), _as_vec3(M)))
        return out

    for item in _iter_values(shaft_loads):
        if hasattr(item, "F_world") and hasattr(item, "M_world"):
            shaft = str(getattr(item, "shaft", getattr(item, "name", "shaft")))
            out.append(_ShaftLoadVec(shaft, _as_vec3(item.F_world), _as_vec3(item.M_world)))
            continue

        if isinstance(item, dict):
            F = item.get("F_world", item.get("F", None))
            M = item.get("M_world", item.get("M", None))
            if F is None or M is None:
                continue
            shaft = str(item.get("shaft", item.get("name", "shaft")))
            out.append(_ShaftLoadVec(shaft, _as_vec3(F), _as_vec3(M)))
            continue

        if isinstance(item, (tuple, list)) and len(item) == 2:
            try:
                out.append(_ShaftLoadVec("shaft", _as_vec3(item[0]), _as_vec3(item[1])))
            except Exception:
                continue

    return out


def _normalize_bearing_loads(bearing_loads: Any) -> list[_LoadVec]:
    """
    Accepts multiple shapes:
      - list[BearingLoad] where BearingLoad has .bearing and .R_world
      - dict[bearing -> vec3]
      - list[dict] with keys {"bearing","R_world"} or {"name","R_world"}
      - list[(name, vec3)]
    """
    out: list[_LoadVec] = []
    if isinstance(bearing_loads, dict):
        for k, v in bearing_loads.items():
            try:
                out.append(_LoadVec(str(k), _as_vec3(v)))
            except Exception:
                continue
        return out

    for item in _iter_values(bearing_loads):
        if hasattr(item, "R_world"):
            name = str(getattr(item, "bearing", getattr(item, "name", "bearing")))
            out.append(_LoadVec(name, _as_vec3(getattr(item, "R_world"))))
            continue

        if isinstance(item, dict):
            if "R_world" not in item:
                continue
            name = str(item.get("bearing", item.get("name", "bearing")))
            out.append(_LoadVec(name, _as_vec3(item["R_world"])))
            continue

        if isinstance(item, (tuple, list)) and len(item) >= 2:
            name = str(item[0])
            try:
                out.append(_LoadVec(name, _as_vec3(item[1])))
            except Exception:
                continue

    return out


# ------------------------------- demo main -------------------------------

def main() -> None:
    m = build_model()

    # Example pose + load case
    q = np.array([0.0, 0.3, 0.5], dtype=float)          # rad
    F_tip = np.array([0.0, 0.0, 20.0], dtype=float)     # N (up +Z)
    preload = 5.0                                      # N

    # World state (for tip pose display)
    (
        _frames,
        tip_pose,
        _wpulleys,
        _wendpoints,
        _wdrums,
        _wshafts,
        _wbearings,
    ) = m.world_state(q)

    # Full pipeline
    out = analyze_tip_force(m, q, F_tip, preload=float(preload))

    tau = np.asarray(out["tau"], float).reshape(-1)
    A = np.asarray(out["A"], float)
    T = np.asarray(out["tensions"], float).reshape(-1)
    tau_hat = np.asarray(out["tau_hat"], float).reshape(-1)

    nnls_err = float(out.get("nnls_err", np.linalg.norm(A @ T - tau)))

    pulley_loads = out.get("pulley_loads", [])
    shaft_loads = out.get("shaft_loads", [])
    bearing_loads = out.get("bearing_loads", [])

    bearing_lives = out.get("bearing_lives", [])
    shaft_stresses = out.get("shaft_stresses", [])

    # ---- Print report ----
    print("\n================== rds_finger demo_all ==================")
    print("Model summary:")
    print(f"  DOF count: {q.size}")
    print(f"  Tendons:   {len(m.tendon_order)}  ({', '.join(m.tendon_order)})")
    print(f"  Shafts:    {len(m.shafts)}")
    print(f"  Pulleys:   {len(m.pulleys)}")
    print(f"  Bearings:  {len(getattr(m, 'bearings', {}))}")
    print("")

    print("Input:")
    print(f"  q (rad):      {q}")
    print(f"  F_tip_xyz:    {_fmt_vec3(F_tip, 'N')}")
    if tip_pose is not None:
        p = getattr(tip_pose, "p", None)
        R = getattr(tip_pose, "R", None)
        if p is not None:
            print(f"  tip position: {_fmt_vec3(p, 'mm')}")
        if R is not None:
            print(f"  tip R:\n{_fmt_mat(R)}")
    print("")

    print("Statics core:")
    print(f"  tau_ref (N*mm): {_fmt_vec3(tau, 'N*mm')}")
    print("  A(q) (N*mm per N):")
    print(_fmt_mat(A))
    print("")

    print("Solved tendon tensions (N):")
    for name, tval in zip(m.tendon_order, T):
        print(f"  {name:>10s}: {float(tval):9.3f}")
    print("")

    print("Reconstruction check:")
    denom = 1.0 + float(np.linalg.norm(tau))
    rel = float(np.linalg.norm(tau_hat - tau)) / denom
    print(f"  ||A T - tau|| = {nnls_err:.6g} (N*mm)")
    print(f"  relative error = {rel:.6g}")
    print(f"  tau_hat (N*mm): {_fmt_vec3(tau_hat, 'N*mm')}")
    print("")

    feas = out.get("feasibility", None)
    if isinstance(feas, dict) and "rows" in feas:
        print("Feasibility diagnostics (per joint row):")
        for r in feas["rows"]:
            if not isinstance(r, dict):
                print(f"  {r}")
                continue
            i = r.get("row", "?")
            status = str(r.get("status", ""))
            t = r.get("tau", None)
            th = r.get("tau_hat", None)
            if t is not None and th is not None:
                print(f"  row {i}: {status:>22s}  tau={float(t): .3f}  tau_hat={float(th): .3f}")
            else:
                print(f"  row {i}: {status}")
        print("")

    print("Loads summary:")
    # For display, count normalized items so you don't get fooled by dict-vs-list.
    pl_norm = _normalize_pulley_loads(pulley_loads)
    sl_norm = _normalize_shaft_loads(shaft_loads)
    bl_norm = _normalize_bearing_loads(bearing_loads)
    print(f"  pulley loads:  {len(pl_norm)}")
    print(f"  shaft loads:   {len(sl_norm)}")
    print(f"  bearing loads: {len(bl_norm)}")
    print("")

    if bearing_lives:
        worst = worst_bearing_life(bearing_lives)
        if worst is not None:
            print("Bearing life (simple L10) worst-case:")
            print(f"  bearing: {worst.bearing}  shaft: {worst.shaft}")
            print(f"  P = {worst.P_N:.3f} N   C = {worst.C_N:.3f} N   p = {worst.p:g}")
            print(f"  L10 = {worst.L10_rev:.3e} rev")
            print("")

    if shaft_stresses:
        worstS = worst_shaft_stress(shaft_stresses)
        if worstS is not None:
            print("Shaft stress (bending-only) worst-case:")
            print(f"  shaft: {worstS.shaft}")
            print(f"  d = {worstS.diameter_mm:.3f} mm")
            print(f"  |M| = {worstS.M_Nmm:.3f} N*mm")
            print(f"  sigma_b = {worstS.sigma_b_MPa:.3f} MPa")
            print(f"  von_mises (bending-only) = {worstS.von_mises_MPa:.3f} MPa")
            print("")

    # ---- Sanity checks (raise loudly if anything is broken) ----
    print("Sanity checks:")

    assert np.isfinite(tau).all()
    assert np.isfinite(A).all()
    assert np.isfinite(T).all()
    assert (T >= -1e-9).all()
    print("  finite + nonnegative tensions: OK")

    for pl in pl_norm:
        assert np.isfinite(pl.vec).all()
    for sl in sl_norm:
        assert np.isfinite(sl.F).all()
        assert np.isfinite(sl.M).all()
    for bl in bl_norm:
        assert np.isfinite(bl.vec).all()
    print("  loads finite: OK")

    print("\nDone.\n")


if __name__ == "__main__":
    main()