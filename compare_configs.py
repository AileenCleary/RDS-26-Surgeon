import math

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt

from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from scipy.optimize import linprog, minimize
from scipy.spatial import ConvexHull

from route_configs import CONFIG_LIST
import csv

def origin_in_hull(S):
    """Determine tensionability of a given structure matrix S.
    
    I.e., is the origin enclosed by the convex hull of the polytope which
        is formed by the column-space of S,
        and contains all the possible joint torques.

    Each col of S is a torque vector per unit tendon tension.

    Solves: "Does a tension vector T exist such that ST = 0 (zero joint torque)?"
        Additionally constrained by T >= 0 (no negative tensions), and
        the sum of vector T elements = 1 (normalized tension distribution for T_sum = 1 N).

    Normalization (sum(T)=1):
        Without normalization constraint, T=0 would always solve ST=0, meaning
        that the origin is always achievable (incorrect), so constraint removes trivial solution and
        makes the set of possible torques a bounded polytope (convex hull) instead of an unbounded cone.
    """
    S = np.asarray(S, float)
    if S.shape[0] != 3:
        raise ValueError(f"S must be 3×n, got {S.shape}")

    n = S.shape[1]
    res = linprog(
        c=np.zeros(n),
        A_eq=np.vstack([S, np.ones((1, n))]),
        b_eq=np.array([0.0, 0.0, 0.0, 1.0]),
        bounds=[(0.0, None)] * n,
        method="highs",
    )
    return res.success, (res.x if res.success else None)


def solve_t_star(S):
    """Return t* and one optimal T*.

    Maximize t s.t.
        S@T = 0
        sum(T)=1
        T_i >= t  for all i
        T >= 0, t >= 0

    t* > 0  => origin is strictly inside (not just on a face)
    t* = 0  => feasible, but at least one tendon must go slack
    """
    S = np.asarray(S, float)
    n = S.shape[1]

    # decision vars: x = [T0..T(n-1), t]
    c = np.zeros(n + 1)
    c[-1] = -1.0  # maximize t == minimize -t

    A_eq = np.zeros((4, n + 1))
    A_eq[:3, :n] = S
    A_eq[3, :n] = 1.0
    b_eq = np.array([0.0, 0.0, 0.0, 1.0])

    # T_i >= t  <=>  -T_i + t <= 0
    A_ub = np.zeros((n, n + 1))
    A_ub[:, :n] = -np.eye(n)
    A_ub[:, -1] = 1.0
    b_ub = np.zeros(n)

    res = linprog(
        c=c,
        A_ub=A_ub, b_ub=b_ub,
        A_eq=A_eq, b_eq=b_eq,
        bounds=[(0.0, None)] * n + [(0.0, None)],
        method="highs",
    )
    if not res.success:
        return None, None

    T_star = np.asarray(res.x[:n], float)
    t_star = float(res.x[-1])
    return t_star, T_star


def normalized_robustness(t_star, n):
    """rho = n*t*, since best-case t* is 1/n."""
    if t_star is None:
        return None
    return float(n) * float(t_star)


def distance_to_hull(S):
    """If outside: minimize ||S@T|| with T>=0 and sum(T)=1."""
    S = np.asarray(S, float)
    n = S.shape[1]
    x0 = np.ones(n) / n

    def obj(T):
        v = S @ T
        return float(v @ v)

    res = minimize(
        obj,
        x0=x0,
        method="SLSQP",
        bounds=[(0.0, 1.0)] * n,
        constraints=[{"type": "eq", "fun": lambda T: np.sum(T) - 1.0}],
        options={"ftol": 1e-12, "maxiter": 500},
    )
    if not res.success:
        return None
    return float(np.linalg.norm(S @ res.x))


def plot_hull(ax, S, color):
    """Draw convex hull of columns of S and return points + volume."""
    P = np.asarray(S, float).T
    hull = ConvexHull(P)

    faces = [P[s] for s in hull.simplices]
    ax.add_collection3d(
        Poly3DCollection(faces, facecolor=color, edgecolor=color, alpha=0.25)
    )

    ax.scatter(P[:, 0], P[:, 1], P[:, 2], s=20, color=color)
    ax.scatter([0], [0], [0], marker="x", s=70, color="k")

    return P, float(hull.volume)


def set_equal_limits(axes, points_list):
    """Make all subplots use the same axis limits (so comparisons are fair)."""
    P = np.vstack(points_list)
    mins, maxs = P.min(axis=0), P.max(axis=0)
    center = (mins + maxs) / 2.0
    span = float((maxs - mins).max())

    for ax in axes:
        ax.set_xlim(center[0] - span / 2, center[0] + span / 2)
        ax.set_ylim(center[1] - span / 2, center[1] + span / 2)
        ax.set_zlim(center[2] - span / 2, center[2] + span / 2)
        ax.set_box_aspect([1, 1, 1])


def fmt(x):
    return "—" if x is None else f"{x:.4g}"


def bottleneck_indices(T_star, t_star, eps=1e-6):
    """Which tendons are actually limiting t* (the ones near the minimum)."""
    if T_star is None or t_star is None:
        return []
    T_star = np.asarray(T_star, float).reshape(-1)

    # keep it relative so this behaves even if t* is tiny
    tol = max(float(eps), 1e-3 * float(max(1.0, abs(t_star))))
    return np.where(T_star <= t_star + tol)[0].tolist()


def hull_face_margin_to_origin(S):
    """How deep the origin is inside the hull (geometric margin).

    If inside, each hull face is a plane. For each face plane, measure distance
    from origin to that plane, then take the minimum, i.e., the smallest "gap" from origin to any hull face.
    Larger margin => origin is deeper => more geometric slack.
    """
    P = np.asarray(S, float).T

    # if hull is degenerate (points coplanar etc), ConvexHull can explode
    try:
        hull = ConvexHull(P)
    except Exception:
        return None

    best = np.inf

    # convex polytope can be written as an intersection of halfspaces
    # hull.equations gives outward normals directly:
    #   n·x + d = 0 for points on face
    #   n·x + d <= 0 for points inside the hull
    for eq in hull.equations:
        n = eq[:3]
        d = float(eq[3])

        nn = float(np.linalg.norm(n))
        if nn < 1e-12:
            continue

        # distance from origin to plane is |d|/||n||.
        # for inside points, origin should satisfy d <= 0, so distance = -d/||n||.
        dist = (-d) / nn
        if dist < best:
            best = dist

    if not np.isfinite(best):
        return None
    return float(best)

def normalized_hull_volume(S):
    P = np.asarray(S, float).T
    V = float(ConvexHull(P).volume)
    norms = np.linalg.norm(P, axis=1)
    scale = float(np.mean(norms))
    if scale < 1e-12:
        return None
    return V / (scale**3)

from scipy.optimize import linprog

def support_radius(S, u):
    S = np.asarray(S, float)
    u = np.asarray(u, float).reshape(3)
    n = S.shape[1]
    # maximize u^T S T  == minimize -(u^T S) T
    c = -(u @ S)  # shape (n,)
    A_eq = np.ones((1, n))
    b_eq = np.array([1.0])
    res = linprog(c=c, A_eq=A_eq, b_eq=b_eq, bounds=[(0.0, None)]*n, method="highs")
    if not res.success:
        return None
    return -float(res.fun)

def random_unit_vectors(m, rng=np.random.default_rng(0)):
    X = rng.normal(size=(m, 3))
    X /= np.linalg.norm(X, axis=1, keepdims=True)
    return X

def directional_capability_metrics(S, m=400, seed=0):
    dirs = random_unit_vectors(m, np.random.default_rng(seed))
    rs = []
    for u in dirs:
        r = support_radius(S, u)
        if r is not None:
            rs.append(r)
    rs = np.array(rs, float)
    if rs.size == 0:
        return None
    return {
        "r_mean": float(rs.mean()),
        "r_min": float(rs.min()),
        "r_max": float(rs.max()),
        "isotropy": float(rs.min() / rs.max()) if rs.max() > 1e-12 else None,
    }
VERBOSE = True
VERBOSE2 = True

if __name__ == "__main__":
    """
    inside: origin in hull -> "tension closure": can put tendons under nonnegative tension distribution that
        produces zero net generalized joint torque. All tendons pull, but the torques cancel.
    
    t*: largest guaranteed minimum component of normalized tension distribution T while still achieving zero
        torque. Each tension T_i in T is a fraction of the total tension. For example, if t*=0.06, then there exists
        a zero-torque preload state where every tendon carries at least 6% of the total tension.

    T*: From the t* optimization, represents the best possible "minimum tendon fraction". 
        At optimality, some tendons will hit the lower bound T_i = t*, so those tendons limit how high t can go.
        The routing "cant keep these tendons loaded" while still cancelling the torques, so changing the routing around this -> higher robustness.

    volume: computed from convex hull of points s_i (s_i units of torque/N or torque per "unit tension").
        Volume is a proxy/estimation for how broadly a tendon set can span the 3D torque space under a distributed
        fixed total tension. volume can be large for bad tensionability though.
    """
    configs = list(CONFIG_LIST)

    # grid that fits any number of configs
    k = len(configs)
    cols = int(math.ceil(math.sqrt(k)))
    rows = int(math.ceil(k / cols))

    colors = plt.rcParams["axes.prop_cycle"].by_key().get(
        "color", ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    )

    fig = plt.figure(figsize=(4.5 * cols, 4.0 * rows))
    axes, all_pts, results = [], [], []

    for idx, cfg in enumerate(configs):
        S = np.asarray(cfg.S, float)
        if S.shape[0] != 3:
            raise ValueError(f"{cfg.name}: S must be 3×n, got {S.shape}")

        names = getattr(cfg, "tendon_names", None)
        if names is not None and len(names) != S.shape[1]:
            raise ValueError(
                f"{cfg.name}: tendon_names length {len(names)} != number of columns {S.shape[1]}"
            )

        inside, _T0 = origin_in_hull(S)

        t_star, T_star = solve_t_star(S) if inside else (None, None)
        rho = normalized_robustness(t_star, S.shape[1]) if inside else None

        bottlenecks = bottleneck_indices(T_star, t_star) if inside else []
        margin = hull_face_margin_to_origin(S) if inside else None

        dist = distance_to_hull(S) if not inside else None

        ax = fig.add_subplot(rows, cols, idx + 1, projection="3d")
        axes.append(ax)

        color = colors[idx % len(colors)]
        P, vol = plot_hull(ax, S, color)
        vol_norm = normalized_hull_volume(S)
        t_dict = directional_capability_metrics(S)
        all_pts.append(P)

        ax.set_xlabel("τ₁")
        ax.set_ylabel("τ₂")
        ax.set_zlabel("τ₃")
        ax.view_init(elev=20, azim=35)

        ax.set_title(
            f"{cfg.name}\n"
            f"{'INSIDE' if inside else 'OUTSIDE'} | "
            f"t*={fmt(t_star)} | "
            f"dist={fmt(dist)} | "
            f"Vol={fmt(vol)}",
            fontsize=10,
        )

        results.append({
            "name": cfg.name,
            "description": cfg.desc,
            "inside": inside,
            "t_star": t_star,
            "rho": rho,
            "dist": dist,
            "volume": vol,
            "vol_norm": vol_norm,
            "margin": margin,
            "T_star": T_star,
            "bottlenecks": bottlenecks,
            "tendon_names": names,
            "r_mean": t_dict["r_mean"],
            "r_min": t_dict["r_min"],
            "r_max": t_dict["r_max"],
            "isotropy": t_dict["isotropy"],
        })

    set_equal_limits(axes, all_pts)
    plt.suptitle("Tensionability polytopes (conv hull of columns of S)", y=0.98)
    plt.tight_layout()
    plt.show()

    # rank: inside first, then higher rho, then higher margin, then higher volume
    def rank_key(r):
        if r["inside"]:
            return (
                2,
                r["rho"] if r["rho"] is not None else 0.0,
                r["margin"] if r["margin"] is not None else 0.0,
                r["volume"],
            )
        d = r["dist"] if r["dist"] is not None else 1e9
        return (1, -d, r["volume"])

    ranked = sorted(results, key=rank_key, reverse=True)

    print("\n=== Ranked Comparison ===")

    if VERBOSE:
        for rank, r in enumerate(ranked, 1):
            print(f"\n{rank}) {r['name']}")
            print(f"   {r['description']}")
            print(f"   inside : {r['inside']}")
            print(f"   t*     : {fmt(r['t_star'])}")
            print(f"   rho    : {fmt(r['rho'])}")
            print(f"   dist   : {fmt(r['dist'])}")
            print(f"   volume : {fmt(r['volume'])}")
            print(f"   norm volume : {fmt(r['vol_norm'])}")
            print(f"   margin : {fmt(r['margin'])}")
            print(f"   r_mean : {fmt(r['r_mean'])}")
            print(f"   r_min : {fmt(r['r_min'])}")
            print(f"   r_max  : {fmt(r['r_max'])}")
            print(f"   isotropy : {fmt(r['isotropy'])}")

            if VERBOSE2:
                T = r.get("T_star", None)
                if T is None:
                    continue

                T = np.asarray(T, float).reshape(-1)
                n = T.size

                names = r.get("tendon_names", None)
                if not names:
                    names = [f"tendon_{j}" for j in range(n)]
                else:
                    names = list(names)[:n] + [f"tendon_{j}" for j in range(len(names), n)]

                bn = r.get("bottlenecks", [])
                if bn:
                    print("   bottleneck tendons:")
                    for j in bn:
                        print(f"      - {names[j]:<24}  T = {T[j]:.6g}")

                print("   \noptimal preload distribution (T_star, sum=1):")
                for j in np.argsort(-T):
                    print(f"      {names[j]:<24}  {T[j]:.6g}")
        