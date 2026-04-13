import numpy as np
import matplotlib.pyplot as plt
from rds_finger.config import CCW, CW
from rds_finger.statics.utils import plane_basis_with_vertical, project_to_plane_coords
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.pulleys import Pulley
from rds_finger.statics.tangent import compute_tangents_point_to_pulley, choose_tangent_point_to_pulley, compute_tangents_pulley_to_pulley, choose_tangent_pulley_to_pulley

def line_segment_limits_from_points(
        points2d, 
        pad_frac=0.2
):
    """Plot helper function""" # ***
    pts = np.asarray(points2d, dtype=float)
    mn = pts.min(axis=0)
    mx = pts.max(axis=0)
    span = mx - mn
    pad = pad_frac * max(span[0], span[1], 1.0)
    return mn, mx, pad

def _draw_circle(ax, c, r, **kwargs):
    th = np.linspace(0.0, 2.0 * np.pi, 400)
    x = c[0] + r * np.cos(th)
    y = c[1] + r * np.sin(th)
    ax.plot(x, y, **kwargs)

def _set_limits(ax, points2d, pad_frac=0.2):
    mn, mx, pad = line_segment_limits_from_points(points2d, pad_frac=pad_frac)
    ax.set_xlim(mn[0] - pad, mx[0] + pad)
    ax.set_ylim(mn[1] - pad, mx[1] + pad)

def _format_axis(ax, title):
    ax.set_title(title)
    ax.set_xlabel("plane x")
    ax.set_ylabel("plane y")
    ax.set_aspect("equal")
    ax.grid(True)

def _dir_label(dir_value: int) -> str:
    if dir_value == CW:
        return "CW"
    if dir_value == CCW:
        return "CCW"
    return f"dir={dir_value}"

def _point_marker_label(point: Point3D) -> str:
    if hasattr(point, "type"):
        return point.type
    return "point"

def _pulley_plane_data(pulley: Pulley):
    ex, ey, n = plane_basis_with_vertical(pulley.axis)
    origin = pulley.center
    c2 = np.array([0.0, 0.0], dtype=float)
    return ex, ey, n, origin, c2

def _two_pulley_plane_data(pulley1: Pulley, pulley2: Pulley):
    axis = pulley1.axis if np.dot(pulley1.axis, pulley2.axis) >= 0 else -pulley1.axis
    ex, ey, n = plane_basis_with_vertical(axis)
    origin = pulley1.center

    c1_2d = project_to_plane_coords(pulley1.center, origin, ex, ey)
    c2_2d = project_to_plane_coords(pulley2.center, origin, ex, ey)
    return ex, ey, n, origin, c1_2d, c2_2d

def _line_direction_from_tangent_pair(t1, t2, tol=1e-12):
    d = np.asarray(t2, dtype=float) - np.asarray(t1, dtype=float)
    n = np.linalg.norm(d)
    if n < tol:
        raise ValueError("Degenerate tangent pair: tangent points are identical.")
    return d / n

def plot_point_pulley_all_solutions(
        point: Point3D,
        pulley: Pulley,
        ax=None,
):
    """
    Plot all point->pulley tangent solutions in the pulley's local 2D plane.
    """
    created_fig = False
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 5))
        created_fig = True

    ex, ey, n, origin, c2 = _pulley_plane_data(pulley)
    p2 = project_to_plane_coords(point.center, origin, ex, ey)
    tan_pts = compute_tangents_point_to_pulley(point, pulley)

    pts_for_limits = [c2, p2]

    _draw_circle(ax, c2, pulley.radius, linewidth=1.5)
    ax.scatter([c2[0]], [c2[1]], marker="x", s=70)
    ax.scatter([p2[0]], [p2[1]], marker="o", s=50)

    for i, t in enumerate(tan_pts):
        pts_for_limits.append(t)
        ax.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=1.5, label=f"tangent {i+1}")
        ax.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.0)
        ax.scatter([t[0]], [t[1]], marker="s", s=45)

    _format_axis(
        ax,
        f"All point→pulley tangents\npulley={getattr(pulley, 'name', 'pulley')}, dir={_dir_label(pulley.dir)}"
    )
    _set_limits(ax, pts_for_limits)

    if created_fig:
        fig.tight_layout()
        return fig, ax
    return ax

def plot_point_pulley_chosen_solution(
        point: Point3D,
        pulley: Pulley,
        ax=None,
):
    """
    Plot only the chosen point->pulley tangent solution.
    """
    created_fig = False
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 5))
        created_fig = True

    ex, ey, n, origin, c2 = _pulley_plane_data(pulley)
    p2 = project_to_plane_coords(point.center, origin, ex, ey)
    t = choose_tangent_point_to_pulley(point, pulley)

    pts_for_limits = [c2, p2, t]

    _draw_circle(ax, c2, pulley.radius, linewidth=1.5)
    ax.scatter([c2[0]], [c2[1]], marker="x", s=70)
    ax.scatter([p2[0]], [p2[1]], marker="o", s=50)
    ax.scatter([t[0]], [t[1]], marker="s", s=55)

    ax.plot([p2[0], t[0]], [p2[1], t[1]], linewidth=2.0)
    ax.plot([c2[0], t[0]], [c2[1], t[1]], "--", linewidth=1.0)

    _format_axis(
        ax,
        f"Chosen point→pulley tangent\npulley={getattr(pulley, 'name', 'pulley')}, dir={_dir_label(pulley.dir)}"
    )
    _set_limits(ax, pts_for_limits)

    if created_fig:
        fig.tight_layout()
        return fig, ax
    return ax

def save_point_pulley_all_vs_chosen(
        point: Point3D,
        pulley: Pulley,
        filename: str,
):
    """
    Save a two-panel figure:
      left  = all valid point->pulley tangents
      right = chosen tangent according to pulley.dir
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    plot_point_pulley_all_solutions(point, pulley, ax=axes[0])
    plot_point_pulley_chosen_solution(point, pulley, ax=axes[1])
    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def save_point_pulley_direction_scenarios(
        point: Point3D,
        pulley_center,
        radius,
        axis,
        tendon="unknown",
        shaft="unknown",
        base_name="Pulley",
        filename="point_pulley_direction_scenarios.png",
):
    """
    Save a 2x2 figure:
      row 1 = CW, all / chosen
      row 2 = CCW, all / chosen
    """
    pul_cw = Pulley(
        center=pulley_center,
        radius=radius,
        axis=axis,
        dir=CW,
        tendon=tendon,
        shaft=shaft,
        name=f"{base_name}_CW",
    )
    pul_ccw = Pulley(
        center=pulley_center,
        radius=radius,
        axis=axis,
        dir=CCW,
        tendon=tendon,
        shaft=shaft,
        name=f"{base_name}_CCW",
    )

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    plot_point_pulley_all_solutions(point, pul_cw, ax=axes[0, 0])
    plot_point_pulley_chosen_solution(point, pul_cw, ax=axes[0, 1])
    plot_point_pulley_all_solutions(point, pul_ccw, ax=axes[1, 0])
    plot_point_pulley_chosen_solution(point, pul_ccw, ax=axes[1, 1])

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def plot_pulley_pulley_all_solutions(
        pulley1: Pulley,
        pulley2: Pulley,
        ax=None,
        line_length: float = 100.0,
):
    """
    Plot all pulley->pulley tangent solutions in the common 2D pulley plane.
    """
    created_fig = False
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 5))
        created_fig = True

    ex, ey, n, origin, c1_2d, c2_2d = _two_pulley_plane_data(pulley1, pulley2)
    tan_pairs = compute_tangents_pulley_to_pulley(pulley1, pulley2)

    pts_for_limits = [c1_2d, c2_2d]

    _draw_circle(ax, c1_2d, pulley1.radius, linewidth=1.5)
    _draw_circle(ax, c2_2d, pulley2.radius, linewidth=1.5)
    ax.scatter([c1_2d[0], c2_2d[0]], [c1_2d[1], c2_2d[1]], marker="x", s=70)

    for i, pair in enumerate(tan_pairs):
        t1, t2 = pair
        pts_for_limits.extend([t1, t2])

        d = _line_direction_from_tangent_pair(t1, t2)
        a = t1 - line_length * d
        b = t1 + line_length * d

        ax.plot([a[0], b[0]], [a[1], b[1]], linewidth=1.3, label=f"tangent {i+1}")
        ax.plot([c1_2d[0], t1[0]], [c1_2d[1], t1[1]], "--", linewidth=1.0)
        ax.plot([c2_2d[0], t2[0]], [c2_2d[1], t2[1]], "--", linewidth=1.0)
        ax.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s", s=45)

    _format_axis(
        ax,
        f"All pulley→pulley tangents\n"
        f"{getattr(pulley1, 'name', 'P1')}={_dir_label(pulley1.dir)}, "
        f"{getattr(pulley2, 'name', 'P2')}={_dir_label(pulley2.dir)}"
    )
    _set_limits(ax, pts_for_limits)

    if created_fig:
        fig.tight_layout()
        return fig, ax
    return ax

def plot_pulley_pulley_chosen_solution(
        pulley1: Pulley,
        pulley2: Pulley,
        ax=None,
        line_length: float = 100.0,
):
    """
    Plot only the chosen pulley->pulley tangent solution.
    """
    created_fig = False
    if ax is None:
        fig, ax = plt.subplots(figsize=(6, 5))
        created_fig = True

    ex, ey, n, origin, c1_2d, c2_2d = _two_pulley_plane_data(pulley1, pulley2)
    t1, t2 = choose_tangent_pulley_to_pulley(pulley1, pulley2)

    d = _line_direction_from_tangent_pair(t1, t2)
    a = t1 - line_length * d
    b = t1 + line_length * d

    pts_for_limits = [c1_2d, c2_2d, t1, t2]

    _draw_circle(ax, c1_2d, pulley1.radius, linewidth=1.5)
    _draw_circle(ax, c2_2d, pulley2.radius, linewidth=1.5)
    ax.scatter([c1_2d[0], c2_2d[0]], [c1_2d[1], c2_2d[1]], marker="x", s=70)
    ax.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s", s=55)

    ax.plot([a[0], b[0]], [a[1], b[1]], linewidth=2.0)
    ax.plot([c1_2d[0], t1[0]], [c1_2d[1], t1[1]], "--", linewidth=1.0)
    ax.plot([c2_2d[0], t2[0]], [c2_2d[1], t2[1]], "--", linewidth=1.0)

    _format_axis(
        ax,
        f"Chosen pulley→pulley tangent\n"
        f"{getattr(pulley1, 'name', 'P1')}={_dir_label(pulley1.dir)}, "
        f"{getattr(pulley2, 'name', 'P2')}={_dir_label(pulley2.dir)}"
    )
    _set_limits(ax, pts_for_limits)

    if created_fig:
        fig.tight_layout()
        return fig, ax
    return ax

def save_pulley_pulley_all_vs_chosen(
        pulley1: Pulley,
        pulley2: Pulley,
        filename: str,
):
    """
    Save a two-panel figure:
      left  = all valid pulley->pulley tangents
      right = chosen tangent according to pulley1.dir and pulley2.dir
    """
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    plot_pulley_pulley_all_solutions(pulley1, pulley2, ax=axes[0])
    plot_pulley_pulley_chosen_solution(pulley1, pulley2, ax=axes[1])
    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

def save_pulley_pulley_direction_scenarios(
        pulley1_center,
        r1,
        pulley2_center,
        r2,
        axis,
        tendon="unknown",
        shaft1="shaft1",
        shaft2="shaft2",
        base_name1="P1",
        base_name2="P2",
        filename="pulley_pulley_direction_scenarios.png",
):
    """
    Save a 4x2 figure:
      rows = (CW,CW), (CW,CCW), (CCW,CW), (CCW,CCW)
      cols = all / chosen
    """
    combos = [
        (CW,  CW),
        (CW,  CCW),
        (CCW, CW),
        (CCW, CCW),
    ]

    fig, axes = plt.subplots(len(combos), 2, figsize=(12, 5 * len(combos)))

    for row, (d1, d2) in enumerate(combos):
        p1 = Pulley(
            center=pulley1_center,
            radius=r1,
            axis=axis,
            dir=d1,
            tendon=tendon,
            shaft=shaft1,
            name=f"{base_name1}_{_dir_label(d1)}",
        )
        p2 = Pulley(
            center=pulley2_center,
            radius=r2,
            axis=axis,
            dir=d2,
            tendon=tendon,
            shaft=shaft2,
            name=f"{base_name2}_{_dir_label(d2)}",
        )

        plot_pulley_pulley_all_solutions(p1, p2, ax=axes[row, 0])
        plot_pulley_pulley_chosen_solution(p1, p2, ax=axes[row, 1])

    fig.tight_layout()
    fig.savefig(filename, dpi=220, bbox_inches="tight")
    plt.close(fig)

