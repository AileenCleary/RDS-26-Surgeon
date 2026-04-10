import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple

GLOBAL_UP = np.array([0.0,0.0,1.0]) # global +z

def unit(
        v: np.ndarray, 
        tol: float = 1e-12,
) -> np.ndarray:
    """Convert input vector v to a unit vector."""
    v = np.asarray(v, dtype=float) # make sure v is a numpy array
    n = np.linalg.norm(v) # calculate length of vector
    
    if n < tol:
        raise ValueError("Zero-length vector.") # if length is basically 0, return an error
    
    return v / n # return v divided by its length ns

def rot90(
        v_2d: np.ndarray,
) -> np.ndarray:
    """Rotate a 2D vector v_2d by +90 degrees to find perpendicular directions when computing tangent points."""
    v_2d = np.asarray(v_2d, dtype=float) # make sure v_2d is a numpy array of floats

    # rotates vector [x,y] by +90 degrees to [-y,x]
    return np.array([-v_2d[1], v_2d[0]], dtype=float)

def plane_basis_with_vertical(
        axis: np.ndarray, 
        tol: float = 1e-12
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build orthonormal coordinate axis for pulley plane."""
    n = unit(axis)
    v = np.asarray(GLOBAL_UP, dtype=float)

    # project global up into (pulley plane) where 
    # so the projection of v onto plane orthogonal to n is calculated by subtracting
    # v's component along n from v, leaving a vector parallel to the plane
    # i.e., y = v - (v dot n)n
    ey = v - np.dot(v, n) * n

    ey = unit(y) # normalize vertical plane axis
    ex = unit(np.cross(ey, n)) # cross product to get perp plane horizontal axis
    ey = unit(np.cross(n, ex)) 

    return ex, ey, n

def project_to_plane_coords(
        p3: np.ndarray, 
        origin3: np.ndarray, 
        ex: np.ndarray, 
        ey: np.ndarray,
) -> np.ndarray:
    r = np.asarray(p3, dtype=float) - np.asarray(origin3, dtype=float)
    return np.array([np.dot(r, ex), np.dot(r, ey)], dtype=float)

def lift_from_plane_coords(p2, origin3, ex, ey):
    p2 = np.asarray(p2, dtype=float)
    return np.asarray(origin3, dtype=float) + p2[0] * ex + p2[1] * ey

def line_segment_limits_from_points(points2d, pad_frac=0.2):
    pts = np.asarray(points2d, dtype=float)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    span = mx - mn
    pad = pad_frac * max(span[0], span[1], 1.0)
    return mn, mx, pad


# ============================================================
# Data classes
# ============================================================

@dataclass
class Point3D:
    center: np.ndarray
    name: str = "Point"

    def __post_init__(self):
        self.center = np.asarray(self.center, dtype=float).reshape(3)

@dataclass
class Pulley:
    center: np.ndarray
    radius: float
    axis: np.ndarray
    direction: str = "CW"      # CW -> choose lower, CCW -> choose upper
    name: str = "Pulley"

    def __post_init__(self):
        self.center = np.asarray(self.center, dtype=float).reshape(3)
        self.radius = float(self.radius)
        self.axis = unit(np.asarray(self.axis, dtype=float).reshape(3))
        if self.radius <= 0:
            raise ValueError("Pulley radius must be positive.")
        if self.direction not in ("CW", "CCW"):
            raise ValueError("direction must be 'CW' or 'CCW'.")


# ============================================================
# Core 2D tangent solvers
# ============================================================

def tangents_point_circle_2d(p, c, r, tol=1e-12):
    """
    Tangency points on circle (c, r) from external point p.
    Returns one or two 2D tangent points on the circle.
    """
    p = np.asarray(p, dtype=float)
    c = np.asarray(c, dtype=float)

    u = p - c
    d2 = np.dot(u, u)
    d = np.sqrt(d2)

    if d < r - tol:
        raise ValueError("Point lies inside circle: no real tangents.")
    if abs(d - r) < tol:
        return [p.copy()]

    a = r * r / d2
    b = r * np.sqrt(d2 - r * r) / d2
    up = rot90(u)

    t1 = c + a * u + b * up
    t2 = c + a * u - b * up
    return [t1, t2]

def tangents_circle_circle_2d(c1, r1, c2, r2, tol=1e-12):
    """
    All common tangents between circles (c1, r1) and (c2, r2).
    Returns list of dicts with:
      t1, t2, direction, family
    """
    c1 = np.asarray(c1, dtype=float)
    c2 = np.asarray(c2, dtype=float)

    d = c2 - c1
    D = np.linalg.norm(d)
    if D < tol:
        raise ValueError("Coincident circle centers: degenerate case.")

    e = d / D
    ep = rot90(e)
    sols = []

    for s in (+1, -1):  # +1 external, -1 internal
        a = (r1 - s * r2) / D
        if abs(a) > 1 + tol:
            continue
        a = np.clip(a, -1.0, 1.0)
        b = np.sqrt(max(0.0, 1.0 - a * a))

        branches = [1] if b < tol else [1, -1]
        for sign in branches:
            n = unit(a * e + sign * b * ep)
            t1 = c1 + r1 * n
            t2 = c2 + s * r2 * n
            dline = unit(rot90(n))
            sols.append({
                "t1": t1,
                "t2": t2,
                "direction": dline,
                "family": "external" if s == 1 else "internal",
            })

    return sols


# ============================================================
# 3D wrappers with projection into pulley plane
# ============================================================

def tangents_point_pulley(point, pulley, vertical_hint=None):
    """
    Compute point-pulley tangents by projecting to the pulley plane.
    """
    ex, ey, n = plane_basis_with_vertical(pulley.axis, vertical_hint=vertical_hint)
    origin = pulley.center

    p2 = project_to_plane_coords(point.center, origin, ex, ey)
    c2 = np.array([0.0, 0.0], dtype=float)

    t_points = tangents_point_circle_2d(p2, c2, pulley.radius)
    sols = []
    for t2 in t_points:
        sols.append({
            "point_2d": p2,
            "center_2d": c2,
            "tangent_point_2d": t2,
            "point_3d": point.center,
            "center_3d": pulley.center,
            "tangent_point_3d": lift_from_plane_coords(t2, origin, ex, ey),
            "basis": (ex, ey, n),
            "origin": origin,
            "vertical_3d": ey,
        })
    return sols

def tangents_pulley_pulley(p1, p2, vertical_hint=None, axis_tol=1e-9):
    """
    Compute pulley-pulley tangents by projecting to their shared plane.
    Assumes pulley axes are parallel or anti-parallel.
    """
    if np.linalg.norm(np.cross(p1.axis, p2.axis)) > axis_tol:
        raise ValueError("Pulley axes must be parallel or anti-parallel.")

    axis = p1.axis if np.dot(p1.axis, p2.axis) >= 0 else -p1.axis
    ex, ey, n = plane_basis_with_vertical(axis, vertical_hint=vertical_hint)
    origin = p1.center

    c1_2d = project_to_plane_coords(p1.center, origin, ex, ey)
    c2_2d = project_to_plane_coords(p2.center, origin, ex, ey)

    sols2d = tangents_circle_circle_2d(c1_2d, p1.radius, c2_2d, p2.radius)
    sols = []
    for s in sols2d:
        sols.append({
            "c1_2d": c1_2d,
            "c2_2d": c2_2d,
            "t1_2d": s["t1"],
            "t2_2d": s["t2"],
            "t1_3d": lift_from_plane_coords(s["t1"], origin, ex, ey),
            "t2_3d": lift_from_plane_coords(s["t2"], origin, ex, ey),
            "direction_2d": s["direction"],
            "family": s["family"],
            "basis": (ex, ey, n),
            "origin": origin,
            "vertical_3d": ey,
        })
    return sols


# ============================================================
# Choice rules
# ============================================================

def preferred_sign_from_direction(direction):
    # CW chooses lower -> negative vertical coordinate
    # CCW chooses upper -> positive vertical coordinate
    return -1 if direction == "CW" else +1

def signed_height_from_center(t2, c2):
    return float(t2[1] - c2[1])

def choose_point_pulley_solution(point, pulley, vertical_hint=None):
    sols = tangents_point_pulley(point, pulley, vertical_hint=vertical_hint)
    desired = preferred_sign_from_direction(pulley.direction)

    best = None
    best_score = -np.inf
    for s in sols:
        h = signed_height_from_center(s["tangent_point_2d"], s["center_2d"])
        score = desired * h
        if score > best_score:
            best_score = score
            best = s
    return best

def choose_pulley_pulley_solution(p1, p2, vertical_hint=None):
    sols = tangents_pulley_pulley(p1, p2, vertical_hint=vertical_hint)
    desired1 = preferred_sign_from_direction(p1.direction)
    desired2 = preferred_sign_from_direction(p2.direction)

    best = None
    best_score = -np.inf
    for s in sols:
        h1 = signed_height_from_center(s["t1_2d"], s["c1_2d"])
        h2 = signed_height_from_center(s["t2_2d"], s["c2_2d"])
        score = desired1 * h1 + desired2 * h2
        if score > best_score:
            best_score = score
            best = s
    return best


# ============================================================
# Verification helpers
# ============================================================

def verify_point_circle_2d(p, c, r, t, atol=1e-9):
    on_circle = np.isclose(np.linalg.norm(t - c), r, atol=atol)
    tangent = np.isclose(np.dot(t - c, p - t), 0.0, atol=atol)
    return on_circle and tangent

def verify_circle_circle_2d(c1, r1, c2, r2, t1, t2, atol=1e-9):
    ok1 = np.isclose(np.linalg.norm(t1 - c1), r1, atol=atol)
    ok2 = np.isclose(np.linalg.norm(t2 - c2), r2, atol=atol)
    ok3 = np.isclose(np.dot(t1 - c1, t2 - t1), 0.0, atol=atol)
    ok4 = np.isclose(np.dot(t2 - c2, t2 - t1), 0.0, atol=atol)
    return ok1 and ok2 and ok3 and ok4


# ============================================================
# Plotting helpers
# Only save images; do not show
# Vertical axis in figures is the chosen "up" direction
# ============================================================

def _draw_circle(ax, c, r, **kwargs):
    th = np.linspace(0, 2 * np.pi, 400)
    xy = np.column_stack([c[0] + r * np.cos(th), c[1] + r * np.sin(th)])
    ax.plot(xy[:, 0], xy[:, 1], **kwargs)

def _format_axis(ax, title, vertical_label="vertical (+up)"):
    ax.set_title(title)
    ax.set_xlabel("horizontal")
    ax.set_ylabel(vertical_label)
    ax.set_aspect("equal")
    ax.grid(True)

def _set_limits(ax, points2d):
    mn, mx, pad = line_segment_limits_from_points(points2d)
    ax.set_xlim(mn[0] - pad, mx[0] + pad)
    ax.set_ylim(mn[1] - pad, mx[1] + pad)

def _direction_text(vertical_3d):
    v = np.asarray(vertical_3d, dtype=float)
    return f"selection vertical = +{np.array2string(v, precision=3, suppress_small=True)}"

def save_point_pulley_solutions_and_choice(point, pulley, filename, vertical_hint=None):
    """
    One figure with:
      left: all possible point-pulley tangents
      right: chosen tangent based on pulley.direction
    """
    sols = tangents_point_pulley(point, pulley, vertical_hint=vertical_hint)
    chosen = choose_point_pulley_solution(point, pulley, vertical_hint=vertical_hint)

    c2 = sols[0]["center_2d"]
    p2 = sols[0]["point_2d"]
    vertical_text = _direction_text(sols[0]["vertical_3d"])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax_all, ax_chosen = axes

    all_points = [c2, p2]
    for s in sols:
        all_points.append(s["tangent_point_2d"])
        t = s["tangent_point_2d"]
        ax_all.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=1.5)
        ax_all.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.0)
        ax_all.scatter([t[0]], [t[1]], marker="s")

    _draw_circle(ax_all, c2, pulley.radius)
    ax_all.scatter([p2[0]], [p2[1]], marker="o", label=point.name)
    ax_all.scatter([c2[0]], [c2[1]], marker="x", label=pulley.name)
    _format_axis(
        ax_all,
        f"All point-pulley solutions\n{pulley.name} direction={pulley.direction}\n{vertical_text}"
    )
    _set_limits(ax_all, all_points)

    t = chosen["tangent_point_2d"]
    ax_chosen.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=2.0)
    ax_chosen.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.2)
    _draw_circle(ax_chosen, c2, pulley.radius)
    ax_chosen.scatter([p2[0]], [p2[1]], marker="o")
    ax_chosen.scatter([c2[0]], [c2[1]], marker="x")
    ax_chosen.scatter([t[0]], [t[1]], marker="s", s=60)
    _format_axis(
        ax_chosen,
        f"Chosen point-pulley solution\nrule: CW=lower, CCW=upper"
    )
    _set_limits(ax_chosen, all_points)

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def save_point_pulley_direction_scenarios(point, pulley_center, radius, axis, filename, vertical_hint=None):
    """
    One figure with two rows:
      row 1: CW case -> [all, chosen]
      row 2: CCW case -> [all, chosen]
    """
    pul_cw = Pulley(pulley_center, radius, axis, "CW", "Pulley_CW")
    pul_ccw = Pulley(pulley_center, radius, axis, "CCW", "Pulley_CCW")

    scenarios = [pul_cw, pul_ccw]
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    for row, pul in enumerate(scenarios):
        sols = tangents_point_pulley(point, pul, vertical_hint=vertical_hint)
        chosen = choose_point_pulley_solution(point, pul, vertical_hint=vertical_hint)

        c2 = sols[0]["center_2d"]
        p2 = sols[0]["point_2d"]
        vertical_text = _direction_text(sols[0]["vertical_3d"])

        ax_all = axes[row, 0]
        ax_chosen = axes[row, 1]

        all_points = [c2, p2]
        for s in sols:
            all_points.append(s["tangent_point_2d"])
            t = s["tangent_point_2d"]
            ax_all.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=1.5)
            ax_all.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.0)
            ax_all.scatter([t[0]], [t[1]], marker="s")

        _draw_circle(ax_all, c2, pul.radius)
        ax_all.scatter([p2[0]], [p2[1]], marker="o")
        ax_all.scatter([c2[0]], [c2[1]], marker="x")
        _format_axis(
            ax_all,
            f"{pul.direction}: all solutions\n{vertical_text}"
        )
        _set_limits(ax_all, all_points)

        t = chosen["tangent_point_2d"]
        ax_chosen.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=2.0)
        ax_chosen.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.2)
        _draw_circle(ax_chosen, c2, pul.radius)
        ax_chosen.scatter([p2[0]], [p2[1]], marker="o")
        ax_chosen.scatter([c2[0]], [c2[1]], marker="x")
        ax_chosen.scatter([t[0]], [t[1]], marker="s", s=60)
        _format_axis(
            ax_chosen,
            f"{pul.direction}: chosen\nrule: CW=lower, CCW=upper"
        )
        _set_limits(ax_chosen, all_points)

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def save_pulley_pulley_solutions_and_choice(p1, p2, filename, vertical_hint=None):
    """
    One figure with:
      left: all possible pulley-pulley tangents
      right: chosen tangent based on each pulley's direction
    """
    sols = tangents_pulley_pulley(p1, p2, vertical_hint=vertical_hint)
    chosen = choose_pulley_pulley_solution(p1, p2, vertical_hint=vertical_hint)

    c1 = sols[0]["c1_2d"]
    c2 = sols[0]["c2_2d"]
    vertical_text = _direction_text(sols[0]["vertical_3d"])

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    ax_all, ax_chosen = axes

    all_points = [c1, c2]
    for s in sols:
        all_points.extend([s["t1_2d"], s["t2_2d"]])
        t1 = s["t1_2d"]
        t2 = s["t2_2d"]
        d = s["direction_2d"]
        L = 100.0
        a = t1 - L * d
        b = t1 + L * d
        ax_all.plot([a[0], b[0]], [a[1], b[1]], linewidth=1.3)
        ax_all.plot([c1[0], t1[0]], [c1[1], t1[1]], "--", linewidth=1.0)
        ax_all.plot([c2[0], t2[0]], [c2[1], t2[1]], "--", linewidth=1.0)
        ax_all.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s")

    _draw_circle(ax_all, c1, p1.radius)
    _draw_circle(ax_all, c2, p2.radius)
    ax_all.scatter([c1[0], c2[0]], [c1[1], c2[1]], marker="x")
    _format_axis(
        ax_all,
        f"All pulley-pulley solutions\n{p1.name}:{p1.direction}, {p2.name}:{p2.direction}\n{vertical_text}"
    )
    _set_limits(ax_all, all_points)

    t1 = chosen["t1_2d"]
    t2 = chosen["t2_2d"]
    d = chosen["direction_2d"]
    L = 100.0
    a = t1 - L * d
    b = t1 + L * d

    ax_chosen.plot([a[0], b[0]], [a[1], b[1]], linewidth=2.0)
    ax_chosen.plot([c1[0], t1[0]], [c1[1], t1[1]], "--", linewidth=1.2)
    ax_chosen.plot([c2[0], t2[0]], [c2[1], t2[1]], "--", linewidth=1.2)
    _draw_circle(ax_chosen, c1, p1.radius)
    _draw_circle(ax_chosen, c2, p2.radius)
    ax_chosen.scatter([c1[0], c2[0]], [c1[1], c2[1]], marker="x")
    ax_chosen.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s", s=60)
    _format_axis(
        ax_chosen,
        "Chosen pulley-pulley solution\nrule: CW=lower, CCW=upper"
    )
    _set_limits(ax_chosen, all_points)

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def save_pulley_pulley_direction_scenarios(p1_center, r1, p2_center, r2, axis, filename, vertical_hint=None):
    """
    One figure with multiple scenarios.
    Each row is one direction combination:
      [all solutions, chosen solution]
    """
    combos = [
        ("CW",  "CW"),
        ("CW",  "CCW"),
        ("CCW", "CW"),
        ("CCW", "CCW"),
    ]

    fig, axes = plt.subplots(len(combos), 2, figsize=(12, 5 * len(combos)))

    for row, (d1, d2) in enumerate(combos):
        p1 = Pulley(p1_center, r1, axis, d1, "P1")
        p2 = Pulley(p2_center, r2, axis, d2, "P2")

        sols = tangents_pulley_pulley(p1, p2, vertical_hint=vertical_hint)
        chosen = choose_pulley_pulley_solution(p1, p2, vertical_hint=vertical_hint)

        c1 = sols[0]["c1_2d"]
        c2 = sols[0]["c2_2d"]
        vertical_text = _direction_text(sols[0]["vertical_3d"])

        ax_all = axes[row, 0]
        ax_chosen = axes[row, 1]

        all_points = [c1, c2]
        for s in sols:
            all_points.extend([s["t1_2d"], s["t2_2d"]])
            t1 = s["t1_2d"]
            t2 = s["t2_2d"]
            d = s["direction_2d"]
            L = 100.0
            a = t1 - L * d
            b = t1 + L * d
            ax_all.plot([a[0], b[0]], [a[1], b[1]], linewidth=1.3)
            ax_all.plot([c1[0], t1[0]], [c1[1], t1[1]], "--", linewidth=1.0)
            ax_all.plot([c2[0], t2[0]], [c2[1], t2[1]], "--", linewidth=1.0)
            ax_all.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s")

        _draw_circle(ax_all, c1, p1.radius)
        _draw_circle(ax_all, c2, p2.radius)
        ax_all.scatter([c1[0], c2[0]], [c1[1], c2[1]], marker="x")
        _format_axis(
            ax_all,
            f"{d1}/{d2}: all solutions\n{vertical_text}"
        )
        _set_limits(ax_all, all_points)

        t1 = chosen["t1_2d"]
        t2 = chosen["t2_2d"]
        d = chosen["direction_2d"]
        L = 100.0
        a = t1 - L * d
        b = t1 + L * d

        ax_chosen.plot([a[0], b[0]], [a[1], b[1]], linewidth=2.0)
        ax_chosen.plot([c1[0], t1[0]], [c1[1], t1[1]], "--", linewidth=1.2)
        ax_chosen.plot([c2[0], t2[0]], [c2[1], t2[1]], "--", linewidth=1.2)
        _draw_circle(ax_chosen, c1, p1.radius)
        _draw_circle(ax_chosen, c2, p2.radius)
        ax_chosen.scatter([c1[0], c2[0]], [c1[1], c2[1]], marker="x")
        ax_chosen.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s", s=60)
        _format_axis(
            ax_chosen,
            f"{d1}/{d2}: chosen\nrule: CW=lower, CCW=upper"
        )
        _set_limits(ax_chosen, all_points)

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)


# ============================================================
# Tests
# ============================================================

def test_point_circle_2d():
    p = np.array([6.0, 2.0], dtype=float)
    c = np.array([0.0, 0.0], dtype=float)
    r = 1.5
    sols = tangents_point_circle_2d(p, c, r)
    assert len(sols) == 2
    assert all(verify_point_circle_2d(p, c, r, t) for t in sols)
    print("test_point_circle_2d passed")

def test_circle_circle_2d():
    c1 = np.array([0.0, 0.0], dtype=float)
    c2 = np.array([6.0, 2.0], dtype=float)
    r1, r2 = 1.5, 1.0
    sols = tangents_circle_circle_2d(c1, r1, c2, r2)
    assert len(sols) == 4
    assert all(verify_circle_circle_2d(c1, r1, c2, r2, s["t1"], s["t2"]) for s in sols)
    print("test_circle_circle_2d passed")

def test_vertical_basis_y_axis_gives_z_up():
    axis = np.array([0.0, 1.0, 0.0], dtype=float)
    ex, ey, n = plane_basis_with_vertical(axis, vertical_hint=np.array([0.0, 0.0, 1.0]))
    # up should be +z
    assert np.allclose(ey, np.array([0.0, 0.0, 1.0]), atol=1e-9)
    print("test_vertical_basis_y_axis_gives_z_up passed")

def test_3d_point_pulley_projection():
    point = Point3D([6.0, 0.0, 2.0], "A")
    pulley = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "P")
    sols = tangents_point_pulley(point, pulley)
    assert len(sols) == 2
    for s in sols:
        tp = s["tangent_point_3d"]
        assert np.isclose(np.dot(tp - pulley.center, pulley.axis), 0.0, atol=1e-9)
        assert np.isclose(np.linalg.norm(tp - pulley.center), pulley.radius, atol=1e-9)
    print("test_3d_point_pulley_projection passed")

def test_3d_pulley_pulley_projection():
    p1 = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "P1")
    p2 = Pulley([6.0, 0.0, 2.0], 1.0, [0.0, 1.0, 0.0], "CCW", "P2")
    sols = tangents_pulley_pulley(p1, p2)
    assert len(sols) == 4
    for s in sols:
        assert np.isclose(np.linalg.norm(s["t1_3d"] - p1.center), p1.radius, atol=1e-9)
        assert np.isclose(np.linalg.norm(s["t2_3d"] - p2.center), p2.radius, atol=1e-9)
    print("test_3d_pulley_pulley_projection passed")

def test_choice_rule_point_pulley():
    point = Point3D([6.0, 0.0, 2.0], "A")
    pul_cw = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "Pcw")
    pul_ccw = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CCW", "Pccw")

    s1 = choose_point_pulley_solution(point, pul_cw)
    s2 = choose_point_pulley_solution(point, pul_ccw)

    assert s1["tangent_point_2d"][1] <= s2["tangent_point_2d"][1] + 1e-12
    print("test_choice_rule_point_pulley passed")

def test_choice_rule_pulley_pulley():
    p1 = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "P1")
    p2 = Pulley([6.0, 0.0, 2.0], 1.0, [0.0, 1.0, 0.0], "CCW", "P2")
    s = choose_pulley_pulley_solution(p1, p2)
    assert s is not None
    print("test_choice_rule_pulley_pulley passed")

def run_all_tests():
    test_point_circle_2d()
    test_circle_circle_2d()
    test_vertical_basis_y_axis_gives_z_up()
    test_3d_point_pulley_projection()
    test_3d_pulley_pulley_projection()
    test_choice_rule_point_pulley()
    test_choice_rule_pulley_pulley()
    print("All tests passed.")


# ============================================================
# Demo / example runner
# Saves figures only
# ============================================================

def run_examples():
    point = Point3D([6.0, 0.0, 2.0], "PointA")
    pulley = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "PulleyA")

    save_point_pulley_solutions_and_choice(
        point,
        pulley,
        filename="example_point_pulley_all_vs_chosen.png"
    )

    save_point_pulley_direction_scenarios(
        point=point,
        pulley_center=[0.0, 0.0, 0.0],
        radius=1.5,
        axis=[0.0, 1.0, 0.0],
        filename="example_point_pulley_direction_scenarios.png"
    )

    # Example 2: pulley-pulley, both axes along y -> vertical should again be z
    p1 = Pulley([0.0, 0.0, 0.0], 1.5, [0.0, 1.0, 0.0], "CW", "P1")
    p2 = Pulley([6.0, 0.0, 2.0], 1.0, [0.0, 1.0, 0.0], "CCW", "P2")

    save_pulley_pulley_solutions_and_choice(
        p1,
        p2,
        filename="example_pulley_pulley_all_vs_chosen.png"
    )

    save_pulley_pulley_direction_scenarios(
        p1_center=[0.0, 0.0, 0.0],
        r1=1.5,
        p2_center=[6.0, 0.0, 2.0],
        r2=1.0,
        axis=[0.0, 1.0, 0.0],
        filename="example_pulley_pulley_direction_scenarios.png"
    )

    print("Saved example_point_pulley_all_vs_chosen.png")
    print("Saved example_point_pulley_direction_scenarios.png")
    print("Saved example_pulley_pulley_all_vs_chosen.png")
    print("Saved example_pulley_pulley_direction_scenarios.png")


if __name__ == "__main__":
    run_all_tests()
    run_examples()