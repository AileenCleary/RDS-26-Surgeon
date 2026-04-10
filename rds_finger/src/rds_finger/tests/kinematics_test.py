import modern_robotics as md
import numpy as np
import pandas as pd
from scipy.optimize import linprog, minimize
from Tensionability.route_configs import CONFIG_OPTION_3A

LENGTH_DEFAULT = 30.0
RADII_MM = np.arange(2.0, 10.01, 0.5)
CSV_OUT = "motor_radius_results.csv"
CSV_MIN_TORQUE_OUT = "minimum_motor_torque_by_radius.csv"

MOTOR_OPTIONS = [
    ["DCX_22", 640.0],
    ["DCX_26", 653.0],
    ["ECX_FLAT_22", 625.0],
    ["ECX_TORQUE_22", 690.8],
    ["ECX_FLAT_32", 691.5],
    ["EC_i_30", 1785.95],
]

ALPHA = 0.03
MAIN_TREF = 20.0
INTERNAL_TREF = 8.0
MAIN_WEIGHT = 1.0
INTERNAL_WEIGHT = 0.35
MAIN_TMIN = 5.0
INTERNAL_TMIN = 2.0
INTERNAL_FORCE_PENALTY = 0.25
ZERO_TOL = 1e-6

np.set_printoptions(precision=3, suppress=True, linewidth=120)

L_splay = L_proximal = L_middle = LENGTH_DEFAULT
L_distal = 0.5 * LENGTH_DEFAULT

z_hat = np.array([0.0, 0.0, 1.0])
y_hat = np.array([0.0, 1.0, 0.0])

x_mcp = L_splay
x_pip = L_splay + L_proximal
x_dip = L_splay + L_proximal + L_middle

q_splay = np.array([0.0, 0.0, 0.0])
q_mcp = np.array([x_mcp, 0.0, 0.0])
q_pip = np.array([x_pip, 0.0, 0.0])

def screw_axis(omega, q):
    return np.r_[omega, -np.cross(omega, q)]

Slist = np.column_stack([
    screw_axis(z_hat, q_splay),
    screw_axis(y_hat, q_mcp),
    screw_axis(y_hat, q_pip),
])

Js = md.JacobianSpace(Slist, np.zeros(3))

f_tip = np.array([0.0, 0.0, 20.0])
r_tip = np.array([x_dip + 0.5 * L_distal, 0.0, 0.0])
Fs_tip = np.r_[np.cross(r_tip, f_tip), f_tip]
tau_joint = Js.T @ Fs_tip

S = np.asarray(CONFIG_OPTION_3A.S, dtype=float)
tendon_names = list(CONFIG_OPTION_3A.tendon_names)
n = len(tendon_names)

is_internal = np.array(["INTERNAL" in name.upper() for name in tendon_names], dtype=bool)
T_ref = np.where(is_internal, INTERNAL_TREF, MAIN_TREF).astype(float)
weights = np.where(is_internal, INTERNAL_WEIGHT, MAIN_WEIGHT).astype(float)
T_min = np.where(is_internal, INTERNAL_TMIN, MAIN_TMIN).astype(float)

def solve_min_peak(S, tau, T_min):
    m, n = S.shape
    c = np.zeros(n + 1)
    c[-1] = 1.0

    A_eq = np.zeros((m, n + 1))
    A_eq[:, :n] = S
    b_eq = tau

    A_ub = np.zeros((n, n + 1))
    b_ub = np.zeros(n)
    for i in range(n):
        A_ub[i, i] = 1.0
        A_ub[i, -1] = -1.0

    bounds = [(float(T_min[i]), None) for i in range(n)] + [(0.0, None)]

    res = linprog(
        c,
        A_ub=A_ub,
        b_ub=b_ub,
        A_eq=A_eq,
        b_eq=b_eq,
        bounds=bounds,
        method="highs",
    )
    if not res.success:
        return None, None
    return res.x[:-1], float(res.x[-1])

def solve_balanced(S, tau, Tmax, T_min, T_ref, weights, is_internal):
    def obj(T):
        d = T - T_ref
        return (
            ALPHA * np.sum(T)
            + 0.5 * np.sum(weights * d * d)
            + INTERNAL_FORCE_PENALTY * np.sum(T[is_internal])
        )

    def grad(T):
        return (
            ALPHA * np.ones_like(T)
            + weights * (T - T_ref)
            + INTERNAL_FORCE_PENALTY * is_internal.astype(float)
        )

    cons = [{
        "type": "eq",
        "fun": lambda T: S @ T - tau,
        "jac": lambda T: S,
    }]

    bounds = [(float(T_min[i]), float(Tmax)) for i in range(len(T_min))]
    x0 = np.clip(T_ref, T_min, Tmax)

    res = minimize(
        obj,
        x0=x0,
        jac=grad,
        constraints=cons,
        bounds=bounds,
        method="SLSQP",
    )
    if not res.success:
        return None
    return res.x

print("\nJs:\n", Js)
print("\ntau_joint (N*mm):", tau_joint)

rows = []
min_torque_rows = []

for radius_mm in RADII_MM:
    T_peak, z_star = solve_min_peak(S, tau_joint, T_min)

    if z_star is None:
        min_torque_rows.append({
            "radius_mm": radius_mm,
            "min_required_motor_torque_nmm": np.nan,
            "min_required_tension_N": np.nan,
        })
        continue

    min_required_motor_torque = radius_mm * z_star

    min_torque_rows.append({
        "radius_mm": radius_mm,
        "min_required_motor_torque_nmm": min_required_motor_torque,
        "min_required_tension_N": z_star,
    })

    for motor_name, motor_torque_nmm in MOTOR_OPTIONS:
        Tmax = motor_torque_nmm / radius_mm

        row = {
            "motor": motor_name,
            "motor_torque_nmm": motor_torque_nmm,
            "radius_mm": radius_mm,
            "Tmax_limit_N": Tmax,
            "min_required_tension_N": z_star,
            "min_required_motor_torque_nmm": min_required_motor_torque,
            "feasible_by_min_peak": Tmax >= z_star - 1e-9,
        }

        if Tmax < z_star - 1e-9:
            row.update({
                "balanced_success": False,
                "max_tension_N": np.nan,
                "mean_tension_N": np.nan,
                "std_tension_N": np.nan,
                "spread_N": np.nan,
                "num_zero": np.nan,
                "internal_mean_N": np.nan,
                "noninternal_mean_N": np.nan,
                "margin_to_limit_N": np.nan,
                "utilization": np.nan,
                "score": np.inf,
            })
            for name in tendon_names:
                row[f"T_{name}"] = np.nan
            rows.append(row)
            continue

        T = solve_balanced(S, tau_joint, Tmax, T_min, T_ref, weights, is_internal)

        if T is None:
            row.update({
                "balanced_success": False,
                "max_tension_N": np.nan,
                "mean_tension_N": np.nan,
                "std_tension_N": np.nan,
                "spread_N": np.nan,
                "num_zero": np.nan,
                "internal_mean_N": np.nan,
                "noninternal_mean_N": np.nan,
                "margin_to_limit_N": np.nan,
                "utilization": np.nan,
                "score": np.inf,
            })
            for name in tendon_names:
                row[f"T_{name}"] = np.nan
            rows.append(row)
            continue

        max_t = float(np.max(T))
        mean_t = float(np.mean(T))
        std_t = float(np.std(T))
        spread = float(np.max(T) - np.min(T))
        num_zero = int(np.sum(T <= ZERO_TOL))
        internal_mean = float(np.mean(T[is_internal])) if np.any(is_internal) else 0.0
        noninternal_mean = float(np.mean(T[~is_internal])) if np.any(~is_internal) else 0.0
        margin = float(Tmax - max_t)
        utilization = float(max_t / Tmax) if Tmax > 0 else np.inf
        margin_ratio = float(margin / Tmax) if Tmax > 0 else 0.0
        internal_max = float(np.max(T[is_internal])) if np.any(is_internal) else 0.0
        score = (
            1.2 * max_t
            + 0.6 * std_t
            + 25.0 * num_zero
            + 0.4 * internal_max
            + 0.1 * np.sum(T)
        )

        row.update({
            "balanced_success": True,
            "max_tension_N": max_t,
            "mean_tension_N": mean_t,
            "std_tension_N": std_t,
            "spread_N": spread,
            "num_zero": num_zero,
            "internal_mean_N": internal_mean,
            "noninternal_mean_N": noninternal_mean,
            "margin_to_limit_N": margin,
            "utilization": utilization,
            "score": score,
        })

        for name, val in zip(tendon_names, T):
            row[f"T_{name}"] = float(val)

        rows.append(row)

df = pd.DataFrame(rows)
df = df.sort_values(
    ["balanced_success", "score", "max_tension_N", "std_tension_N"],
    ascending=[False, True, True, True]
)
df.to_csv(CSV_OUT, index=False)

df_min_torque = pd.DataFrame(min_torque_rows)
df_min_torque = df_min_torque.sort_values("radius_mm")
df_min_torque.to_csv(CSV_MIN_TORQUE_OUT, index=False)

feasible = df[df["balanced_success"]].copy()

print(f"\nSaved full sweep to: {CSV_OUT}")
print(f"Saved minimum required motor torques by radius to: {CSV_MIN_TORQUE_OUT}")
print(f"feasible combos: {len(feasible)} / {len(df)}")

if len(feasible) > 0:
    best = feasible.iloc[0]
    print("\nBEST PRACTICAL COMBO")
    print("motor:", best["motor"])
    print("radius_mm:", best["radius_mm"])
    print("motor_torque_nmm:", round(best["motor_torque_nmm"], 3))
    print("Tmax_limit_N:", round(best["Tmax_limit_N"], 3))
    print("max_tension_N:", round(best["max_tension_N"], 3))
    print("std_tension_N:", round(best["std_tension_N"], 3))
    print("num_zero:", int(best["num_zero"]))
    print("internal_mean_N:", round(best["internal_mean_N"], 3))
    print("margin_to_limit_N:", round(best["margin_to_limit_N"], 3))
    print("score:", round(best["score"], 3))

    print("\nBest tension vector:")
    for name in tendon_names:
        print(f"  {name}: {best[f'T_{name}']:.3f}")

print("\nMINIMUM REQUIRED MOTOR TORQUE BY RADIUS")
print(df_min_torque.to_string(index=False))