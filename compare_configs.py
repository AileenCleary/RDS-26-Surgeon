import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from route_configs import CONFIG_LIST


COLORS = ["tab:blue", "tab:green", "tab:red", "tab:purple"]


def solve_origin_feasible(S):
    S = np.asarray(S, dtype=float)
    m, n = S.shape

    res = linprog(
        c=np.zeros(n),
        A_eq=np.vstack([S, np.ones((1, n))]),
        b_eq=np.array([0.0, 0.0, 0.0, 1.0]),
        bounds=[(0, None)] * n,
        method="highs",
    )
    return res.success, (res.x if res.success else None)


def solve_robustness(S):
    S = np.asarray(S, dtype=float)
    m, n = S.shape

    c = np.zeros(n + 1)
    c[-1] = -1.0  # maximize t

    A_eq = np.zeros((4, n + 1))
    A_eq[:3, :n] = S
    A_eq[3, :n] = 1.0
    b_eq = np.array([0.0, 0.0, 0.0, 1.0])

    A_ub = np.zeros((n, n + 1))
    A_ub[:, :n] = -np.eye(n)
    A_ub[:, -1] = 1.0
    b_ub = np.zeros(n)

    bounds = [(0, None)] * n + [(0, None)]

    res = linprog(
        c=c,
        A_ub=A_ub, b_ub=b_ub,
        A_eq=A_eq, b_eq=b_eq,
        bounds=bounds,
        method="highs",
    )

    if not res.success:
        return None, None

    return float(res.x[-1]), res.x[:n]


def solve_distance(S):
    S = np.asarray(S, dtype=float)
    m, n = S.shape
    x0 = np.ones(n) / n

    def obj(T):
        v = S @ T
        return float(v @ v)

    cons = [{"type": "eq", "fun": lambda T: np.sum(T) - 1.0}]
    bounds = [(0.0, 1.0)] * n

    res = minimize(
        obj, x0=x0,
        method="SLSQP",
        bounds=bounds,
        constraints=cons,
        options={"ftol": 1e-12, "maxiter": 500},
    )

    if not res.success:
        return None

    return float(np.linalg.norm(S @ res.x))


def add_hull(ax, S, color):
    P = np.asarray(S, dtype=float).T
    hull = ConvexHull(P)

    faces = [P[s] for s in hull.simplices]
    poly = Poly3DCollection(
        faces,
        facecolor=color,
        edgecolor=color,
        alpha=0.25,
    )
    ax.add_collection3d(poly)

    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=20, color=color)
    ax.scatter([0], [0], [0], marker="x", s=70, color="k")

    return P, float(hull.volume)


def set_equal_limits(axs, points):
    P = np.vstack(points)
    mins = P.min(axis=0)
    maxs = P.max(axis=0)
    center = (mins + maxs) / 2.0
    span = (maxs - mins).max()

    for ax in axs:
        ax.set_xlim(center[0] - span / 2, center[0] + span / 2)
        ax.set_ylim(center[1] - span / 2, center[1] + span / 2)
        ax.set_zlim(center[2] - span / 2, center[2] + span / 2)
        ax.set_box_aspect([1, 1, 1])


def fmt(x):
    return "—" if x is None else f"{x:.4g}"


if __name__ == "__main__":

    if len(CONFIG_LIST) != 4:
        raise ValueError("Need exactly 4 configs for 2×2 comparison.")

    fig = plt.figure(figsize=(11, 9))
    axes = []
    all_pts = []
    results = []

    for i, cfg in enumerate(CONFIG_LIST):
        ax = fig.add_subplot(2, 2, i + 1, projection="3d")
        axes.append(ax)

        S = cfg.S
        color = COLORS[i % len(COLORS)]

        inside, T0 = solve_origin_feasible(S)
        t_star, _ = solve_robustness(S) if inside else (None, None)
        dist = solve_distance(S) if not inside else None

        P, vol = add_hull(ax, S, color)
        all_pts.append(P)

        ax.set_xlabel("τ₁")
        ax.set_ylabel("τ₂")
        ax.set_zlabel("τ₃")
        ax.view_init(elev=20, azim=35)

        note = (
            f"{'INSIDE' if inside else 'OUTSIDE'}\n"
            f"t*={fmt(t_star)}\n"
            f"dist={fmt(dist)}\n"
            f"Vol={fmt(vol)}"
        )
        ax.set_title(f"{cfg.name}\n{note}", fontsize=10)

        results.append({
            "name": cfg.name,
            "inside": inside,
            "t_star": t_star,
            "dist": dist,
            "volume": vol,
        })

    set_equal_limits(axes, all_pts)
    plt.suptitle("Tensionability Polytopes (Convex Hull of Columns of S)", y=0.98)
    plt.tight_layout()
    plt.show()

    print("\n=== Ranked Comparison ===")

    def score(r):
        # Inside > Outside
        if r["inside"]:
            return (2, r["t_star"] if r["t_star"] is not None else 0.0, r["volume"])
        else:
            return (1, -r["dist"] if r["dist"] is not None else -1e6, r["volume"])

    ranked = sorted(results, key=score, reverse=True)

    for i, r in enumerate(ranked, 1):
        print(f"\n{i}) {r['name']}")
        print(f"   inside: {r['inside']}")
        print(f"   t*: {fmt(r['t_star'])}")
        print(f"   dist: {fmt(r['dist'])}")
        print(f"   volume: {fmt(r['volume'])}")