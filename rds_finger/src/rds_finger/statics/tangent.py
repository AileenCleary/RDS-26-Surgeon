import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List

from rds_finger.config import CCW, CW
from rds_finger.statics.utils import unit, rot90, plane_basis_with_vertical, project_to_plane_coords, lift_from_plane_coords

from rds_finger.types.pulleys import Pulley
from rds_finger.types.fixed_points import Point3D

def tangents_point_circle_2d(
        p, 
        c, 
        r, 
        tol=1e-12,
):
    """Find the tangent point(s) to a circle from a line with orign external point p."""
    p = np.asarray(p, dtype=float)
    c = np.asarray(c, dtype=float)

    # for a circle with center C(h,k) and radius r, and external point P(x,y):
    # distance to center = sqrt((x-h)**2 + (y-k)**2)
    u = p - c # vector from circle center to point
    d2 = np.dot(u, u)
    d = np.sqrt(d2) # distance/magnitude
    # *** why not just use np.linalg.norm
    
    # checking if point is A) inside circle (shouldn't happen), or B) already on circle, so tangent point is just the original point
    if d < r - tol:
        raise ValueError("Point lies inside circle: no real tangents.")
    if abs(d - r) < tol:
        return [p.copy()]

    # radius of circle is perpendicular to tangent point: (t-c) dot (p-t) = 0
    # *** need to verify solution
    a = r * r / d2 # coefficient of component along u
    b = r * np.sqrt(d2 - r * r) / d2 # coefficient of perpendicular component
    up = rot90(u) # perpendicular vector

    t1 = c + a * u + b * up
    t2 = c + a * u - b * up
    return [t1, t2]

def tangents_circle_circle_2d(
        c1: np.ndarray, 
        r1: float, 
        c2: np.ndarray, 
        r2: float, 
        tol=1e-12,
) -> List[List[float]]:
    """Find all tangent line(s) between two circles."""
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
            sols.append([t1, t2])

    return sols 

def compute_tangents_point_to_pulley(
        point_3d: Point3D, 
        pulley: Pulley,
):
    ex, ey, n = plane_basis_with_vertical(axis=pulley.axis)
    p3 = point_3d.center
    origin = pulley.center
    point_2d = project_to_plane_coords(
        p3=p3,
        origin3=origin,
        ex=ex,
        ey=ey
    )
    plane_origin = np.array([0.0, 0.0], dtype=float)
    # *** wait so in project_to_plane_coords, when we use the pulleys origin as origin3, is 
    # that then considered the "origin" of that plane, so when we solve for tangent points we then use [0,0]?
    tangent_pts_2d = tangents_point_circle_2d(
        p=point_2d,
        c=plane_origin,
        r=pulley.radius
    )
    return tangent_pts_2d # *** returning 2D points! convert to 3d at sm pt if/when needed

def compute_tangents_pulley_to_pulley(
        pulley1,
        pulley2,
):
    a1 = unit(pulley1.axis)
    a2 = unit(pulley2.axis)

    if abs(np.dot(a1, a2)) < 0.999999:
        raise ValueError("Pulleys must have same axis of rotation.\n For transitions between axes (SPLAY -> MCP), must manually add a Point3d.")

    axis = a1
    ex, ey, n = plane_basis_with_vertical(axis=axis)
    origin = pulley1.center

    c1_2d = project_to_plane_coords(
        p3=pulley1.center,
        origin3=origin,
        ex=ex,
        ey=ey
    )
    c2_2d = project_to_plane_coords(
        p3=pulley2.center,
        origin3=origin,
        ex=ex,
        ey=ey
    )

    tangent_pts_2d = tangents_circle_circle_2d(
        c1=c1_2d,
        r1=pulley1.radius,
        c2=c2_2d,
        r2=pulley2.radius
    )

    # *** pts are in 2d
    return tangent_pts_2d

def dist_from_center_signed(
        t,
        c,
):
    return float(t[1] - c[1])

def choose_tangent_point_to_pulley(
        point: Point3D,
        pulley: Pulley,
):
    tan_pts = compute_tangents_point_to_pulley(point, pulley)
    dir = pulley.dir

    best = None
    alpha = -np.inf
    c = np.array([0.0, 0.0], dtype=float)

    for pt in tan_pts:
        ht = dist_from_center_signed(pt, c)
        score = dir * ht
        if score > alpha:
            alpha = score
            best = pt
    
    return best

def choose_tangent_pulley_to_pulley(
        pulley1,
        pulley2,
):
    tan_pts = compute_tangents_pulley_to_pulley(pulley1, pulley2)
    dir1 = pulley1.dir
    dir2 = pulley2.dir
    caxis = pulley1.axis

    axis = pulley1.axis
    ex, ey, n = plane_basis_with_vertical(axis=axis)
    origin = pulley1.center

    c1_2d = project_to_plane_coords(
        p3=pulley1.center,
        origin3=origin,
        ex=ex,
        ey=ey
    )
    c2_2d = project_to_plane_coords(
        p3=pulley2.center,
        origin3=origin,
        ex=ex,
        ey=ey
    )
    best = None
    alpha = -np.inf
    for pts in tan_pts:
        pt1 = pts[0]
        pt2 = pts[1]

        h1 = dist_from_center_signed(pt1, c1_2d)
        h2 = dist_from_center_signed(pt2, c2_2d)
        score = dir1 * h1 + dir2 * h2
        if score > alpha:
            alpha = score
            best = pts
    
    return best

def segment_dir(a, b):
    """
    Return the geometric direction along the tendon segment from element a to element b.

    This is a path-direction helper only.
    It is NOT the same thing as force direction on a pulley.
    """
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        if a.type == "END":
            raise ValueError("Order is sequential. If it begins with a point, a pulley must come after.")

        t_b = chosen_tangent_point_3d(a, b)
        return unit(t_b - a.center)

    if isinstance(a, Pulley) and isinstance(b, Pulley):
        t_a, t_b = chosen_tangent_pair_3d(a, b)
        return unit(t_b - t_a)

    if isinstance(a, Pulley) and isinstance(b, Point3D):
        if b.type == "START":
            raise ValueError("Point must be the end or middle.")

        t_a = chosen_tangent_point_3d(b, a)
        return unit(b.center - t_a)

    raise ValueError("Element was neither Point3D or Pulley.")


def force_dir_on_pulley_from_prev(prev, cur):
    """
    Force direction on current pulley `cur` from the segment connecting prev -> cur.

    This vector points AWAY from the pulley tangent point on `cur`
    toward the previous element.
    """
    if not isinstance(cur, Pulley):
        raise ValueError("cur must be a Pulley.")

    if isinstance(prev, Point3D):
        if prev.type == "END":
            raise ValueError("Order is sequential. A point before a pulley cannot be END.")

        t_cur = chosen_tangent_point_3d(prev, cur)
        return unit(prev.center - t_cur)

    if isinstance(prev, Pulley):
        t_prev, t_cur = chosen_tangent_pair_3d(prev, cur)
        return unit(t_prev - t_cur)

    raise ValueError("prev must be Point3D or Pulley.")


def force_dir_on_pulley_to_next(cur, nxt):
    """
    Force direction on current pulley `cur` from the segment connecting cur -> nxt.

    This vector points AWAY from the pulley tangent point on `cur`
    toward the next element.
    """
    if not isinstance(cur, Pulley):
        raise ValueError("cur must be a Pulley.")

    if isinstance(nxt, Point3D):
        if nxt.type == "START":
            raise ValueError("A point after a pulley cannot be START.")

        t_cur = chosen_tangent_point_3d(nxt, cur)
        return unit(nxt.center - t_cur)

    if isinstance(nxt, Pulley):
        t_cur, t_next = chosen_tangent_pair_3d(cur, nxt)
        return unit(t_next - t_cur)

    raise ValueError("nxt must be Point3D or Pulley.")

def chosen_tangent_point_3d(
        point,
        pulley,
):
    ex, ey, n = plane_basis_with_vertical(pulley.axis)
    t2 = choose_tangent_point_to_pulley(point, pulley)
    return lift_from_plane_coords(t2, pulley.center, ex, ey)

def chosen_tangent_pair_3d(
        pulley1,
        pulley2,
):
    axis = unit(pulley1.axis)
    ex, ey, n = plane_basis_with_vertical(axis)
    origin = pulley1.center

    pts2 = choose_tangent_pulley_to_pulley(pulley1, pulley2)
    t1 = lift_from_plane_coords(pts2[0], origin, ex, ey)
    t2 = lift_from_plane_coords(pts2[1], origin, ex, ey)
    return t1, t2

# if __name__ == "__main__":
#     tendon_tensions = {
#         "PIP_EXT": 0,
#         "PIP_FLX": 124.44,
#         "MCP_FLX": 218.98,
#         "MCP_EXT": 0,
#         "SPLAY_A": 0,
#         "SPLAY_B": 192,
#         "INTERNAL_FLX": 237.5,
#         "INTERNAL_EXT": 9.54
#     }

#     reactions = solve_all_bearing_reactions_from_tendon_tensions(
#         tendon_tensions=tendon_tensions,
#         tendon_paths=TENDON_PATH,
#         shaft_bearings=shaft_bearings,
#         axial_side="left",
#     )

#     print_shaft_reactions(reactions)

#     results = solve_and_check_all_bearings(
#         tendon_tensions=tendon_tensions,
#         tendon_paths=TENDON_PATH,
#         shaft_bearings=shaft_bearings,
#         axial_side="left",
#         rpm=60.0,
#     )

#     print_bearing_results(results)
    # point = Point3D(
    #     center=np.array([6.0, 0.0, 2.0]),
    #     type="anchor",
    #     tendon="flexor",
    # )

    # pulley = Pulley(
    #     center=np.array([0.0, 0.0, 0.0]),
    #     radius=1.5,
    #     axis=np.array([0.0, 1.0, 0.0]),
    #     dir=CW,
    #     tendon="flexor",
    #     shaft="MCP",
    #     name="PulleyA",
    # )

    # save_point_pulley_all_vs_chosen(
    #     point=point,
    #     pulley=pulley,
    #     filename="example_point_pulley_all_vs_chosen.png",
    # )

    # save_point_pulley_direction_scenarios(
    #     point=point,
    #     pulley_center=np.array([0.0, 0.0, 0.0]),
    #     radius=1.5,
    #     axis=np.array([0.0, 1.0, 0.0]),
    #     tendon="flexor",
    #     shaft="MCP",
    #     base_name="PulleyA",
    #     filename="example_point_pulley_direction_scenarios.png",
    # )

    # pulley1 = Pulley(
    #     center=np.array([0.0, 0.0, 0.0]),
    #     radius=1.5,
    #     axis=np.array([0.0, 1.0, 0.0]),
    #     dir=CW,
    #     tendon="flexor",
    #     shaft="MCP",
    #     name="P1",
    # )

    # pulley2 = Pulley(
    #     center=np.array([6.0, 0.0, 2.0]),
    #     radius=1.0,
    #     axis=np.array([0.0, 1.0, 0.0]),
    #     dir=CCW,
    #     tendon="flexor",
    #     shaft="PIP",
    #     name="P2",
    # )

    # save_pulley_pulley_all_vs_chosen(
    #     pulley1=pulley1,
    #     pulley2=pulley2,
    #     filename="example_pulley_pulley_all_vs_chosen.png",
    # )

    # save_pulley_pulley_direction_scenarios(
    #     pulley1_center=np.array([0.0, 0.0, 0.0]),
    #     r1=1.5,
    #     pulley2_center=np.array([6.0, 0.0, 2.0]),
    #     r2=1.0,
    #     axis=np.array([0.0, 1.0, 0.0]),
    #     tendon="flexor",
    #     shaft1="MCP",
    #     shaft2="PIP",
    #     base_name1="P1",
    #     base_name2="P2",
    #     filename="example_pulley_pulley_direction_scenarios.png",
    # )
