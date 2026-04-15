import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  registers '3d' projection
from rds_finger.config import CCW, CW
from rds_finger.statics.utils import plane_basis_with_vertical, project_to_plane_coords, lift_from_plane_coords
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.pulleys import Pulley
from rds_finger.statics.tangent import (
    compute_tangents_point_to_pulley,
    choose_tangent_point_to_pulley,
    compute_tangents_pulley_to_pulley,
    choose_tangent_pulley_to_pulley,
    chosen_tangent_point_3d,
    chosen_tangent_pair_3d,
)

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


# ===================================================================
# 2-D orthographic projection helpers
# ===================================================================

def _proj2d(p3, plane: str) -> np.ndarray:
    """Project a 3D point to 2D for the given orthographic plane ('xy' or 'xz')."""
    p = np.asarray(p3, dtype=float)
    if plane == "xy":
        return p[[0, 1]]
    if plane == "xz":
        return p[[0, 2]]
    raise ValueError(f"plane must be 'xy' or 'xz', got {plane!r}")


def draw_pulley_circle_2d_proj(
        ax,
        pulley: Pulley,
        plane: str = "xy",
        n_pts: int = 200,
        **kwargs,
) -> None:
    """Draw the pulley circle projected orthographically onto xy or xz."""
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)
    th = np.linspace(0.0, 2.0 * np.pi, n_pts)
    pts3 = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    if plane == "xy":
        ax.plot(pts3[:, 0], pts3[:, 1], **kwargs)
    else:
        ax.plot(pts3[:, 0], pts3[:, 2], **kwargs)


def draw_pulley_wrap_arc_2d_proj(
        ax,
        pulley: Pulley,
        t_a: np.ndarray,
        t_b: np.ndarray,
        plane: str = "xy",
        n_pts: int = 120,
        **kwargs,
) -> None:
    """Draw the shorter wrap arc between two tangent touch-points, projected onto xy or xz."""
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)

    def _angle(pt3):
        rel = np.asarray(pt3, dtype=float) - center
        return np.arctan2(np.dot(rel, ey), np.dot(rel, ex))

    a0 = _angle(t_a)
    a1 = _angle(t_b)
    diff = (a1 - a0 + np.pi) % (2.0 * np.pi) - np.pi
    th = np.linspace(a0, a0 + diff, n_pts)
    pts3 = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    if plane == "xy":
        ax.plot(pts3[:, 0], pts3[:, 1], **kwargs)
    else:
        ax.plot(pts3[:, 0], pts3[:, 2], **kwargs)


def draw_segment_2d_proj_all(ax, a, b, plane: str = "xy", **kwargs) -> None:
    """
    Draw every tangent candidate for segment (a -> b) projected onto xy or xz.

    Segment-type handling mirrors draw_segment_3d_all; tangent points are lifted
    to 3D first, then projected.
    """
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        ex, ey, _ = plane_basis_with_vertical(b.axis)
        origin = np.asarray(b.center, dtype=float)
        p2 = _proj2d(a.center, plane)
        for t2d in compute_tangents_point_to_pulley(a, b):
            t = _proj2d(lift_from_plane_coords(t2d, origin, ex, ey), plane)
            ax.plot([p2[0], t[0]], [p2[1], t[1]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Point3D):
        ex, ey, _ = plane_basis_with_vertical(a.axis)
        origin = np.asarray(a.center, dtype=float)
        p2 = _proj2d(b.center, plane)
        for t2d in compute_tangents_point_to_pulley(b, a):
            t = _proj2d(lift_from_plane_coords(t2d, origin, ex, ey), plane)
            ax.plot([t[0], p2[0]], [t[1], p2[1]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Pulley):
        ax1 = np.asarray(a.axis, dtype=float)
        ax2 = np.asarray(b.axis, dtype=float)
        axis = ax1 if np.dot(ax1, ax2) >= 0 else -ax1
        ex, ey, _ = plane_basis_with_vertical(axis)
        origin = np.asarray(a.center, dtype=float)
        for pair in compute_tangents_pulley_to_pulley(a, b):
            t1_2d, t2_2d = pair
            t1 = _proj2d(lift_from_plane_coords(t1_2d, origin, ex, ey), plane)
            t2 = _proj2d(lift_from_plane_coords(t2_2d, origin, ex, ey), plane)
            ax.plot([t1[0], t2[0]], [t1[1], t2[1]], **kwargs)
    # Point3D -> Point3D: skip silently


def draw_segment_2d_proj_chosen(ax, a, b, plane: str = "xy", **kwargs) -> None:
    """Draw only the chosen tangent for segment (a -> b) projected onto xy or xz."""
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        p2 = _proj2d(a.center, plane)
        t  = _proj2d(chosen_tangent_point_3d(a, b), plane)
        ax.plot([p2[0], t[0]], [p2[1], t[1]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Point3D):
        t  = _proj2d(chosen_tangent_point_3d(b, a), plane)
        p2 = _proj2d(b.center, plane)
        ax.plot([t[0], p2[0]], [t[1], p2[1]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Pulley):
        t1_3d, t2_3d = chosen_tangent_pair_3d(a, b)
        t1 = _proj2d(t1_3d, plane)
        t2 = _proj2d(t2_3d, plane)
        ax.plot([t1[0], t2[0]], [t1[1], t2[1]], **kwargs)
    # Point3D -> Point3D: skip silently


def plot_tendon_path_2d_projections(
        tendon_name: str,
        path: list,
        axes=None,
) -> tuple:
    """
    Plot the full tendon path as two orthographic projections side by side.

    Left subplot  = xy plane (top-down view)
    Right subplot = xz plane (side/front view)

    Each panel shows:
      - Pulley circles projected as curves
      - Fixed points (START / MIDDLE / END) as scatter markers
      - All tangent candidates as light dashed lines
      - Chosen path as solid lines
      - Wrap arcs on pulleys touched on both sides

    Parameters
    ----------
    axes : (ax_xy, ax_xz) or None
        Existing axes to draw into.  If None a new 1×2 figure is created.

    Returns
    -------
    (fig, ax_xy, ax_xz)
    """
    if axes is None:
        fig, (ax_xy, ax_xz) = plt.subplots(1, 2, figsize=(14, 6))
    else:
        ax_xy, ax_xz = axes
        fig = ax_xy.get_figure()

    _plane_axes = {"xy": ("x", "y"), "xz": ("x", "z")}
    plane_ax_pairs = [("xy", ax_xy), ("xz", ax_xz)]

    # Pre-compute chosen touch-points for wrap arcs (shared across both projections)
    pulley_touch_points: dict = {}
    world_pts_3d = [np.asarray(item.center, dtype=float) for item in path]

    for idx in range(len(path) - 1):
        a, b = path[idx], path[idx + 1]
        if isinstance(a, Point3D) and isinstance(b, Pulley):
            t3d = chosen_tangent_point_3d(a, b)
            pulley_touch_points.setdefault(idx + 1, []).append(t3d)
            world_pts_3d.extend([np.asarray(a.center, dtype=float), t3d])
        elif isinstance(a, Pulley) and isinstance(b, Point3D):
            t3d = chosen_tangent_point_3d(b, a)
            pulley_touch_points.setdefault(idx, []).append(t3d)
            world_pts_3d.extend([t3d, np.asarray(b.center, dtype=float)])
        elif isinstance(a, Pulley) and isinstance(b, Pulley):
            t1, t2 = chosen_tangent_pair_3d(a, b)
            pulley_touch_points.setdefault(idx, []).append(t1)
            pulley_touch_points.setdefault(idx + 1, []).append(t2)
            world_pts_3d.extend([t1, t2])

    for plane, ax in plane_ax_pairs:
        xlabel, ylabel = _plane_axes[plane]

        # Elements
        for item in path:
            c2 = _proj2d(item.center, plane)
            if isinstance(item, Pulley):
                draw_pulley_circle_2d_proj(ax, item, plane=plane, linewidth=1.2, color="steelblue")
                ax.scatter(c2[0], c2[1], marker="x", s=48, color="steelblue", zorder=3)
                ax.annotate(f" {getattr(item, 'shaft', '?')}", c2, fontsize=7)
            else:
                ax.scatter(c2[0], c2[1], marker="o", s=42, color="firebrick", zorder=3)
                ax.annotate(f" {getattr(item, 'type', 'PT')}", c2, fontsize=7)

        # Tangent lines
        for idx in range(len(path) - 1):
            a, b = path[idx], path[idx + 1]
            draw_segment_2d_proj_all(
                ax, a, b, plane=plane,
                color="lightgray", linestyle="--", linewidth=0.9, alpha=0.6,
            )
            draw_segment_2d_proj_chosen(
                ax, a, b, plane=plane,
                color="C0", linewidth=2.4,
            )

        # Wrap arcs
        for pulley_idx, tangents in pulley_touch_points.items():
            if len(tangents) >= 2:
                draw_pulley_wrap_arc_2d_proj(
                    ax, path[pulley_idx], tangents[0], tangents[1],
                    plane=plane, color="C0", linewidth=1.4, linestyle="--",
                )

        # Axis formatting
        pts_2d = [_proj2d(p, plane) for p in world_pts_3d]
        _set_limits(ax, pts_2d, pad_frac=0.12)
        ax.set_aspect("equal")
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.set_title(f"{plane} projection")
        ax.grid(True)
        if plane == "xy":
            ax.invert_yaxis()  # positive y points down to match physical convention

    fig.suptitle(f"Tendon path: {tendon_name}", fontsize=12)
    fig.tight_layout()
    return fig, ax_xy, ax_xz

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


# ===================================================================
# 3-D helpers
# ===================================================================

def draw_pulley_circle_3d(ax, pulley: Pulley, n_pts: int = 200, **kwargs) -> None:
    """Draw the full circumference of a pulley as a circle in 3D world space."""
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)
    th = np.linspace(0.0, 2.0 * np.pi, n_pts)
    pts = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)


def draw_pulley_wrap_arc_3d(
        ax,
        pulley: Pulley,
        t_a: np.ndarray,
        t_b: np.ndarray,
        n_pts: int = 120,
        **kwargs,
) -> None:
    """Draw the shorter arc on a pulley between two chosen tangent touch-points."""
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)

    def _angle(pt3):
        rel = np.asarray(pt3, dtype=float) - center
        return np.arctan2(np.dot(rel, ey), np.dot(rel, ex))

    a0 = _angle(t_a)
    a1 = _angle(t_b)
    diff = (a1 - a0 + np.pi) % (2.0 * np.pi) - np.pi
    th = np.linspace(a0, a0 + diff, n_pts)
    pts = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)


def _set_axes_equal_3d(ax, points: np.ndarray, pad_frac: float = 0.08) -> None:
    """Set equal-aspect 3D axis limits from a (N,3) array of world points."""
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = 0.5 * (mins + maxs)
    spans = np.maximum(maxs - mins, 1.0)
    radius = 0.5 * spans.max() * (1.0 + pad_frac)
    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def draw_segment_3d_all(ax, a, b, **kwargs) -> None:
    """
    Draw every tangent-line candidate for segment (a -> b) in 3D world space.

    Point3D -> Pulley  : lines from fixed point to each tangent point on the pulley.
    Pulley  -> Point3D : lines from each tangent point on the pulley to fixed point.
    Pulley  -> Pulley  : each candidate tangent segment between the two pulleys.
    Point3D -> Point3D : silently skipped.
    """
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        ex, ey, _ = plane_basis_with_vertical(b.axis)
        origin = np.asarray(b.center, dtype=float)
        candidates = compute_tangents_point_to_pulley(a, b)
        p3 = np.asarray(a.center, dtype=float)
        for t2d in candidates:
            t3d = lift_from_plane_coords(t2d, origin, ex, ey)
            ax.plot([p3[0], t3d[0]], [p3[1], t3d[1]], [p3[2], t3d[2]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Point3D):
        ex, ey, _ = plane_basis_with_vertical(a.axis)
        origin = np.asarray(a.center, dtype=float)
        candidates = compute_tangents_point_to_pulley(b, a)
        p3 = np.asarray(b.center, dtype=float)
        for t2d in candidates:
            t3d = lift_from_plane_coords(t2d, origin, ex, ey)
            ax.plot([t3d[0], p3[0]], [t3d[1], p3[1]], [t3d[2], p3[2]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Pulley):
        ax1 = np.asarray(a.axis, dtype=float)
        ax2 = np.asarray(b.axis, dtype=float)
        axis = ax1 if np.dot(ax1, ax2) >= 0 else -ax1
        ex, ey, _ = plane_basis_with_vertical(axis)
        origin = np.asarray(a.center, dtype=float)
        candidates = compute_tangents_pulley_to_pulley(a, b)
        for pair in candidates:
            t1_2d, t2_2d = pair
            t1_3d = lift_from_plane_coords(t1_2d, origin, ex, ey)
            t2_3d = lift_from_plane_coords(t2_2d, origin, ex, ey)
            ax.plot(
                [t1_3d[0], t2_3d[0]],
                [t1_3d[1], t2_3d[1]],
                [t1_3d[2], t2_3d[2]],
                **kwargs,
            )
    # Point3D -> Point3D: skip silently


def draw_segment_3d_chosen(ax, a, b, **kwargs) -> None:
    """
    Draw only the chosen tangent for segment (a -> b) in 3D world space.

    Point3D -> Pulley  : line from fixed point to chosen tangent point.
    Pulley  -> Point3D : line from chosen tangent point to fixed point.
    Pulley  -> Pulley  : line between the two chosen tangent points.
    Point3D -> Point3D : silently skipped.
    """
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        t3d = chosen_tangent_point_3d(a, b)
        p3 = np.asarray(a.center, dtype=float)
        ax.plot([p3[0], t3d[0]], [p3[1], t3d[1]], [p3[2], t3d[2]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Point3D):
        t3d = chosen_tangent_point_3d(b, a)
        p3 = np.asarray(b.center, dtype=float)
        ax.plot([t3d[0], p3[0]], [t3d[1], p3[1]], [t3d[2], p3[2]], **kwargs)

    elif isinstance(a, Pulley) and isinstance(b, Pulley):
        t1, t2 = chosen_tangent_pair_3d(a, b)
        ax.plot([t1[0], t2[0]], [t1[1], t2[1]], [t1[2], t2[2]], **kwargs)
    # Point3D -> Point3D: skip silently


def plot_tendon_path_3d(
        tendon_name: str,
        path: list,
        ax=None,
        elev: float = 24.0,
        azim: float = -58.0,
) -> tuple:
    """
    Plot the complete 3D tendon path:
      - Pulley circles drawn in their rotation plane
      - Fixed points (START/MIDDLE/END) as scatter markers
      - All tangent candidates as light dashed lines
      - Chosen tangent path as solid lines
      - Wrap arcs on pulleys that the tendon wraps around

    Returns (fig, ax).
    """
    if ax is None:
        fig = plt.figure(figsize=(11, 8))
        ax = fig.add_subplot(111, projection="3d")
    else:
        fig = ax.get_figure()

    world_points = []

    # --- draw elements ---
    for item in path:
        center = np.asarray(item.center, dtype=float)
        world_points.append(center)
        if isinstance(item, Pulley):
            draw_pulley_circle_3d(ax, item, linewidth=1.2, color="steelblue")
            ax.scatter(center[0], center[1], center[2], marker="x", s=48, color="steelblue")
            lbl = f" {getattr(item, 'shaft', '?')}"
            ax.text(center[0], center[1], center[2], lbl, fontsize=7)
        else:
            ax.scatter(center[0], center[1], center[2], marker="o", s=42, color="firebrick")
            lbl = f" {getattr(item, 'type', 'PT')}"
            ax.text(center[0], center[1], center[2], lbl, fontsize=7)

    # --- draw tangent lines and collect chosen touch-points per pulley ---
    pulley_touch_points: dict = {}   # path index -> list of 3D tangent points

    for idx in range(len(path) - 1):
        a, b = path[idx], path[idx + 1]

        draw_segment_3d_all(
            ax, a, b,
            color="lightgray", linestyle="--", linewidth=0.9, alpha=0.6,
        )
        draw_segment_3d_chosen(
            ax, a, b,
            color="C0", linewidth=2.4,
        )

        # record chosen tangent endpoints for wrap-arc computation
        if isinstance(a, Point3D) and isinstance(b, Pulley):
            t3d = chosen_tangent_point_3d(a, b)
            pulley_touch_points.setdefault(idx + 1, []).append(t3d)
            world_points.extend([np.asarray(a.center, dtype=float), t3d])
        elif isinstance(a, Pulley) and isinstance(b, Point3D):
            t3d = chosen_tangent_point_3d(b, a)
            pulley_touch_points.setdefault(idx, []).append(t3d)
            world_points.extend([t3d, np.asarray(b.center, dtype=float)])
        elif isinstance(a, Pulley) and isinstance(b, Pulley):
            t1, t2 = chosen_tangent_pair_3d(a, b)
            pulley_touch_points.setdefault(idx, []).append(t1)
            pulley_touch_points.setdefault(idx + 1, []).append(t2)
            world_points.extend([t1, t2])

    # --- draw wrap arcs on pulleys that are touched on both sides ---
    for pulley_idx, tangents in pulley_touch_points.items():
        if len(tangents) >= 2:
            draw_pulley_wrap_arc_3d(
                ax,
                path[pulley_idx],
                tangents[0],
                tangents[1],
                color="C0",
                linewidth=1.4,
                linestyle="--",
            )

    # --- axis formatting ---
    _set_axes_equal_3d(ax, np.asarray(world_points, dtype=float))
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    ax.set_title(f"Tendon path: {tendon_name}")

    return fig, ax

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

