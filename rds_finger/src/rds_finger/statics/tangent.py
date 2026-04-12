import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
from typing import Tuple, Union

GLOBAL_UP = np.array([0.0,0.0,1.0]) # global +z
CW = -1
CCW = 1

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

# def plane_basis_with_vertical(
#         axis: np.ndarray, 
#         tol: float = 1e-12
# ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
#     """Build orthonormal coordinate axis for pulley plane."""
#     n = unit(axis) # normal axis, i.e., axis of rotation
#     v = np.asarray(GLOBAL_UP, dtype=float) # "global up direction," used for choosing correct tangent point based on CCW/CW

#     # project global up into (pulley plane) where 
#     # so the projection of v onto plane orthogonal to n is calculated by subtracting
#     # v's component along n from v, leaving a vector parallel to the plane
#     # i.e., y = v - (v dot n)n
#     ey = v - np.dot(v, n) * n
#     ey = unit(ey) # normalize vertical plane axis
#     ex = unit(np.cross(ey, n)) # cross product to get perp plane horizontal axis
#     ey = unit(np.cross(n, ex)) # recomputing vertical to verify ortho and RH

#     return ex, ey, n # return = +x (hor), +y (vert), normal/rotational axis
def plane_basis_with_vertical(
        axis: np.ndarray,
        tol: float = 1e-12
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    n = unit(axis)
    v = np.asarray(GLOBAL_UP, dtype=float)

    ey = v - np.dot(v, n) * n

    if np.linalg.norm(ey) < tol:
        v = np.array([1.0, 0.0, 0.0], dtype=float)
        ey = v - np.dot(v, n) * n

        if np.linalg.norm(ey) < tol:
            v = np.array([0.0, 1.0, 0.0], dtype=float)
            ey = v - np.dot(v, n) * n

    ey = unit(ey)
    ex = unit(np.cross(ey, n))
    ey = unit(np.cross(n, ex))

    return ex, ey, n

def project_to_plane_coords(
        p3: np.ndarray, # 3d point
        origin3: np.ndarray, # plane origin point in 3D
        ex: np.ndarray, # plane basis vector
        ey: np.ndarray,
) -> np.ndarray:
    """Project a 3D point into 2D coordinates in a local plane basis."""
    r = np.asarray(p3, dtype=float) - np.asarray(origin3, dtype=float)

    # standard coordinate projection where if
    # r = p3 - origin
    # then
    # x = r dot ex, and y = r dot ey
    return np.array([np.dot(r, ex), np.dot(r, ey)], dtype=float)

def lift_from_plane_coords(
        p2, 
        origin3, 
        ex, 
        ey,
):
    """Project 2D point in local plane basis back into 3D."""
    p2 = np.asarray(p2, dtype=float)
    return np.asarray(origin3, dtype=float) + p2[0] * ex + p2[1] * ey

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

class Bearing:
    def __init__(
            self,
            center: np.ndarray,
            axis: np.ndarray,
            shaft: str,
            side: str,
            c0=None,
            c=None,
            name=None,
    ):
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.axis = np.asarray(axis, dtype=float).reshape(3)
        self.shaft = shaft
        self.side = side
        self.c0 = c0
        self.c = c

        if name is not None:
            self.name = name

class Pulley:
    def __init__(
            self,
            center: np.ndarray,
            radius: float,
            axis: np.ndarray,
            dir: int,
            tendon: str,
            shaft: str,
            name: str | None = None,
    ) -> None:
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.radius = radius
        self.axis = np.asarray(axis, dtype=float).reshape(3)
        self.dir = dir
        self.tendon = tendon
        self.shaft = shaft

        if name is not None:
            self.name = name

class Point3D:
    def __init__(
        self,
        center: np.ndarray,
        type: str,
        tendon: str,
    ) -> None:
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.type = type
        self.tendon = tendon

class TendonContact:
    def __init__(
            self,
            base: Point3D | Pulley,
    ):
        self.base = base
        self.center = base.center
        self.tendon = base.tendon

        if isinstance(self.base, Point3D):
            self.type = base.type
        else:
            self.type = "pulley"

# ***
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

# ***
def tangents_circle_circle_2d(
        c1, 
        r1, 
        c2, 
        r2, 
        tol=1e-12,
):
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
            dline = unit(rot90(n))
            # sols.append({
            #     "t1": t1,
            #     "t2": t2,
            #     "direction": dline,
            #     "family": "external" if s == 1 else "internal",
            # })
            sols.append([t1, t2])

    return sols # *** change to just t1 and t2

def compute_tangents_point_to_pulley(
        point_3d, 
        pulley,
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

# ==========================================================================================================================================

# BLUE
START1 = Point3D(np.array([-25.0, 32.0, 36.0]), "START", "PIP_EXT")
PULLEY1 = Pulley(np.array([7.0, 32.0, 36.0]), 10.0, [0.0, 0.0, 1.0], CCW, "PIP_EXT", "A")
PULLEY2 = Pulley(np.array([26.0, 32.0, 36.0]), 4.15, [0.0, 0.0, 1.0], CW, "PIP_EXT", "0")
MIDDLE1 = Point3D(np.array([33.0, 36.0, 36.0]), "MIDDLE", "PIP_EXT")
PULLEY3 = Pulley(np.array([50.0, 36.0, 22.5]), 13.6, [0.0, 1.0, 0.0], CCW, "PIP_EXT", "1")
PULLEY4 = Pulley(np.array([94.0, 36.0, 22.5]), 10.0, [0.0, 1.0, 0.0], CCW, "PIP_EXT", "2")
END1 = Point3D(np.array([104.0, 36.0, 28]), "END", "PIP_EXT")

# GREEN
START2 = Point3D(np.array([-25.0, 32.0, 30.150]), "START", "PIP_FLX")
PULLEY5 = Pulley(np.array([7.0, 32.0, 30.150]), 10.0, [0.0, 0.0, 1.0], CW, "PIP_FLX", "A")
PULLEY6 = Pulley(np.array([26.0, 32.0, 30.150]), 4.15, [0.0, 0.0, 1.0], CCW, "PIP_FLX", "0")
MIDDLE2 = Point3D(np.array([33.0, 29.35, 30.150]), "MIDDLE", "PIP_FLX")
PULLEY7 = Pulley(np.array([50.0, 29.350, 22.5]), 7.6, [0.0, 1.0, 0.0], CCW, "PIP_FLX", "1")
PULLEY8 = Pulley(np.array([94.0, 29.35, 22.5]), 10.0, [0.0, 1.0, 0.0], CW, "PIP_FLX", "2")
END2 = Point3D(np.array([104.0, 29.35, 28]), "END", "PIP_FLX")

# YELLOW
START3 = Point3D(np.array([-25.0, 32.0, 10.5]), "START", "MCP_FLX")
PULLEY9 = Pulley(np.array([7.0, 32.0, 10.5]), 10.0, [0.0, 0.0, 1.0], CCW, "MCP_FLX", "A")
PULLEY10 = Pulley(np.array([26.0, 32.0, 10.5]), 7.75, [0.0, 0.0, 1.0], CW, "MCP_FLX", "0")
MIDDLE3 = Point3D(np.array([35.0, 32, 22.5]), "MIDDLE", "MCP_FLX")
PULLEY11 = Pulley(np.array([50.0, 40, 22.5]), 13.5, [0.0, 1.0, 0.0], CW, "MCP_FLX", "1")
END3 = Point3D(np.array([64, 40, 30]), "END", "MCP_FLX")

# PURPLE
START4 = Point3D(np.array([-25.0, 32.0, 31.350]), "START", "MCP_EXT")
PULLEY12 = Pulley(np.array([7.0, 32.0, 31.350]), 10.0, [0.0, 0.0, 1.0], CW, "MCP_EXT", "A")
PULLEY13 = Pulley(np.array([26.0, 32.0, 31.350]), 4.15, [0.0, 0.0, 1.0], CCW, "MCP_EXT", "0")
MIDDLE4 = Point3D(np.array([33.0, 27, 31.350]), "MIDDLE", "MCP_EXT")
PULLEY14 = Pulley(np.array([50.0, 27, 22.5]), 10, [0.0, 1.0, 0.0], CCW, "MCP_EXT", "1")
END4 = Point3D(np.array([64, 27, 33]), "END", "MCP_EXT")

# SPLAY A
START5 = Point3D(np.array([-25.0, 32.0, 14]), "START", "SPLAY_A")
PULLEY15 = Pulley(np.array([26, 32.0, 14]), 11.5, [0.0, 0.0, 1.0], CCW, "SPLAY_A", "0")
END5 = Point3D(np.array([12, 32.0, 14]), "END", "SPLAY_A")

# SPLAY B
START6 = Point3D(np.array([-25.0, 32.0, 19]), "START", "SPLAY_B")
PULLEY16 = Pulley(np.array([26, 32.0, 19]), 11.5, [0.0, 0.0, 1.0], CW, "SPLAY_B", "0")
END6 = Point3D(np.array([12, 32.0, 19]), "END", "SPLAY_B")

BEARING_A_L = Bearing(np.array([7.0, 32.0, 41]), [0.0, 0.0, 1.0], "A", "left", 266, 711)
BEARING_A_R = Bearing(np.array([7.0, 32.0, 1]), [0.0, 0.0, 1.0], "A", "right", 266, 711)
BEARING_0_L = Bearing(np.array([26.0, 32.0, 41]), [0.0, 0.0, 1.0], "0","left", 266, 711)
BEARING_0_R = Bearing(np.array([26.0, 32.0, 1]), [0.0, 0.0, 1.0], "0","right", 266, 711)
BEARING_1_L = Bearing(np.array([50, 12, 22.5]), [0.0, 1.0, 0.0], "1","left", 266, 711)
BEARING_1_R = Bearing(np.array([50, 47, 22.5]), [0.0, 1.0, 0.0], "1","right", 266, 711)
BEARING_2_L = Bearing(np.array([94, 12, 22.5]), [0.0, 1.0, 0.0], "2","left", 266, 711)
BEARING_2_R = Bearing(np.array([94, 47, 22.5]), [0.0, 1.0, 0.0], "2","right", 266, 711)

shaft_bearings = {
    "A": {"left": BEARING_A_L, "right": BEARING_A_R},
    "0": {"left": BEARING_0_L, "right": BEARING_0_R},
    "1": {"left": BEARING_1_L, "right": BEARING_1_R},
    "2": {"left": BEARING_2_L, "right": BEARING_2_R},
}

TENDON_PATH = {
    "PIP_EXT": [START1, PULLEY1, PULLEY2, MIDDLE1, PULLEY3, PULLEY4, END1],
    "PIP_FLX": [START2, PULLEY5, PULLEY6, MIDDLE2, PULLEY7, PULLEY8, END2],
    "MCP_FLX": [START3, PULLEY9, PULLEY10, MIDDLE3, PULLEY11, END3],
    "MCP_EXT": [START4, PULLEY12, PULLEY13, MIDDLE4, PULLEY14, END4],
    "SPLAY_A": [START5, PULLEY15, END5],
    "SPLAY_B": [START6, PULLEY16, END6],
}

SHAFT_LOADS = {
    "A": [],
    "0": [],
    "1": [],
    "2": [],
}

class PulleyLoad:
    def __init__(
            self,
            pulley: Pulley,
            v_in,
            v_out,
            t_in=0.0,
            t_out=0.0,
    ):
        self.pulley = pulley
        self.center = pulley.center.copy()
        self.axis = pulley.axis.copy()
        self.shaft = pulley.shaft

        self.v_in = unit(v_in)
        self.v_out = unit(v_out)
        self.t_in = t_in
        self.t_out = t_out

        self.f_in = self.t_in * self.v_in
        self.f_out = self.t_out * self.v_out
        self.f_net = self.f_in + self.f_out

    def set_tensions(
            self,
            t_in,
            t_out,
    ):
        self.t_in = t_in
        self.t_out = t_out
        self.f_in = self.t_in * self.v_in
        self.f_out = self.t_out * self.v_out
        self.f_net = self.f_in + self.f_out

def tan_vector(a, b):
    if isinstance(a, Point3D) and isinstance(b, Pulley):
        if a.type == "END":
            raise ValueError("Order is sequential. If it begins with a point, a pulley must come after.")

        t = chosen_tangent_point_3d(a, b)
        return unit(a.center - t)

    if isinstance(a, Pulley) and isinstance(b, Pulley):
        t1, t2 = chosen_tangent_pair_3d(a, b)
        return unit(t1 - t2)

    if isinstance(a, Pulley) and isinstance(b, Point3D):
        if b.type == "START":
            raise ValueError("Point must be the end or middle.")

        t = chosen_tangent_point_3d(b, a)
        return unit(t - b.center)

    raise ValueError("Element was neither Point3D or Pulley.")

def init_tendon_pulley_loads(
        tendon_path,
        tensions=None,
):
    start = tendon_path[0]
    end = tendon_path[-1]

    if not isinstance(start, Point3D) or not isinstance(end, Point3D):
        raise ValueError("Tendon paths need to start and end with a Point.")
    if (start.type != "START") or (end.type != "END"):
        raise ValueError("Start and end points must be correctly labelled.")

    if tensions is None:
        tensions = [0.0] * (len(tendon_path) - 1)

    for key in SHAFT_LOADS:
        SHAFT_LOADS[key] = []

    for i in range(1, len(tendon_path) - 1):
        prev = tendon_path[i - 1]
        cur = tendon_path[i]
        nxt = tendon_path[i + 1]

        if not isinstance(cur, Pulley):
            continue

        v_in = tan_vector(prev, cur)
        v_out = tan_vector(cur, nxt)

        t_in = tensions[i - 1]
        t_out = tensions[i]

        SHAFT_LOADS[cur.shaft].append(
            PulleyLoad(cur, v_in, v_out, t_in, t_out)
        )

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

def proj(
        v,
        axis,
):
    axis = unit(axis)
    return np.dot(v, axis) * axis


def rej(
        v,
        axis,
):
    return np.asarray(v, dtype=float) - proj(v, axis)


def shaft_coord(
        p,
        origin,
        axis,
):
    return float(np.dot(np.asarray(p, dtype=float) - np.asarray(origin, dtype=float), unit(axis)))

def solve_bearing_reactions(
        left_bearing,
        right_bearing,
        pulley_loads,
        axial_side="left",
):
    axis = unit(left_bearing.axis)
    L = shaft_coord(right_bearing.center, left_bearing.center, axis)

    sum_f_rad = np.zeros(3)
    sum_m_left = np.zeros(3)
    sum_f_ax = 0.0

    for load in pulley_loads:
        x = shaft_coord(load.center, left_bearing.center, axis)

        f = load.f_net
        f_ax = np.dot(f, axis)
        f_rad = f - f_ax * axis
        r = x * axis

        sum_f_rad += f_rad
        sum_m_left += np.cross(r, f_rad)
        sum_f_ax += f_ax

    r_right_rad = -np.cross(axis, sum_m_left) / L
    r_left_rad = -sum_f_rad - r_right_rad

    if axial_side == "left":
        r_left_ax = -sum_f_ax * axis
        r_right_ax = np.zeros(3)
    else:
        r_left_ax = np.zeros(3)
        r_right_ax = -sum_f_ax * axis

    r_left = r_left_rad + r_left_ax
    r_right = r_right_rad + r_right_ax

    return {
        "left_reaction": r_left,
        "right_reaction": r_right,
        "left_radial": r_left_rad,
        "right_radial": r_right_rad,
        "left_axial": r_left_ax,
        "right_axial": r_right_ax,
        "left_radial_mag": np.linalg.norm(r_left_rad),
        "right_radial_mag": np.linalg.norm(r_right_rad),
        "left_axial_mag": np.linalg.norm(r_left_ax),
        "right_axial_mag": np.linalg.norm(r_right_ax),
    }

def bearing_equiv_static_load(
        fr,
        fa,
        x0=0.6,
        y0=0.5,
):
    return x0 * abs(fr) + y0 * abs(fa)


def bearing_equiv_dynamic_load(
        fr,
        fa,
        e=0.30,
        x_low=1.0,
        y_low=0.0,
        x_high=0.56,
        y_high=1.6,
):
    fr = abs(fr)
    fa = abs(fa)

    if fr == 0.0:
        return fa

    if fa / fr <= e:
        return x_low * fr + y_low * fa

    return x_high * fr + y_high * fa


def check_bearing(
        bearing,
        reaction,
        rpm=None,
):
    axis = unit(bearing.axis)
    fa = abs(np.dot(reaction, axis))
    fr = np.linalg.norm(reaction - np.dot(reaction, axis) * axis)

    out = {
        "Fr": fr,
        "Fa": fa,
    }

    if bearing.c0 is not None:
        p0 = bearing_equiv_static_load(fr, fa)
        out["P0"] = p0
        out["static_sf"] = bearing.c0 / p0 if p0 > 0 else np.inf

    if bearing.c is not None:
        p = bearing_equiv_dynamic_load(fr, fa)
        out["P"] = p
        out["dynamic_util"] = p / bearing.c if bearing.c > 0 else np.inf
        out["L10_rev_millions"] = (bearing.c / p) ** 3 if p > 0 else np.inf

        if rpm is not None and rpm > 0:
            out["L10_hours"] = out["L10_rev_millions"] * 1e6 / (60.0 * rpm)

    return out

def solve_all_shaft_reactions(
        shaft_bearings,
        axial_side="left",
):
    out = {}

    for shaft, pulley_loads in SHAFT_LOADS.items():
        if shaft not in shaft_bearings:
            continue

        left_bearing = shaft_bearings[shaft]["left"]
        right_bearing = shaft_bearings[shaft]["right"]

        out[shaft] = solve_bearing_reactions(
            left_bearing,
            right_bearing,
            pulley_loads,
            axial_side=axial_side,
        )

    return out
# ==========================================================================================================================================

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

def segment_tensions_from_value(
        tendon_path,
        tension_value,
):
    nseg = len(tendon_path) - 1

    if np.isscalar(tension_value):
        return [float(tension_value)] * nseg

    vals = list(tension_value)
    if len(vals) != nseg:
        raise ValueError(f"Expected {nseg} segment tensions, got {len(vals)}.")
    return vals


def build_shaft_loads_from_tendon_tensions(
        tendon_tensions,
        tendon_paths,
):
    shaft_loads = {shaft: [] for shaft in SHAFT_LOADS.keys()}

    for tendon_name, tendon_path in tendon_paths.items():
        if tendon_name not in tendon_tensions:
            continue

        tensions = segment_tensions_from_value(
            tendon_path=tendon_path,
            tension_value=tendon_tensions[tendon_name],
        )

        for i in range(1, len(tendon_path) - 1):
            prev = tendon_path[i - 1]
            cur = tendon_path[i]
            nxt = tendon_path[i + 1]

            if not isinstance(cur, Pulley):
                continue

            v_in = tan_vector(prev, cur)
            v_out = tan_vector(cur, nxt)

            t_in = tensions[i - 1]
            t_out = tensions[i]

            shaft_loads[cur.shaft].append(
                PulleyLoad(cur, v_in, v_out, t_in, t_out)
            )

    return shaft_loads


def solve_all_bearing_reactions_from_tendon_tensions(
        tendon_tensions,
        tendon_paths,
        shaft_bearings,
        axial_side="left",
):
    shaft_loads = build_shaft_loads_from_tendon_tensions(
        tendon_tensions=tendon_tensions,
        tendon_paths=tendon_paths,
    )

    reactions = {}

    for shaft, bearings in shaft_bearings.items():
        pulley_loads = shaft_loads.get(shaft, [])

        reactions[shaft] = solve_bearing_reactions(
            left_bearing=bearings["left"],
            right_bearing=bearings["right"],
            pulley_loads=pulley_loads,
            axial_side=axial_side,
        )

    return reactions


def solve_and_check_all_bearings(
        tendon_tensions,
        tendon_paths,
        shaft_bearings,
        axial_side="left",
        rpm=None,
):
    reactions = solve_all_bearing_reactions_from_tendon_tensions(
        tendon_tensions=tendon_tensions,
        tendon_paths=tendon_paths,
        shaft_bearings=shaft_bearings,
        axial_side=axial_side,
    )

    out = {}

    for shaft, res in reactions.items():
        left_bearing = shaft_bearings[shaft]["left"]
        right_bearing = shaft_bearings[shaft]["right"]

        out[shaft] = {
            "reactions": res,
            "left_bearing_check": check_bearing(left_bearing, res["left_reaction"], rpm=rpm),
            "right_bearing_check": check_bearing(right_bearing, res["right_reaction"], rpm=rpm),
        }

    return out

def fmt_vec(v, prec=2):
    v = np.asarray(v, dtype=float)
    return f"[{v[0]:8.{prec}f}, {v[1]:8.{prec}f}, {v[2]:8.{prec}f}]"

def print_shaft_reactions(reactions):
    print("\n" + "="*60)
    print("SHAFT REACTIONS")
    print("="*60)

    for shaft, r in reactions.items():
        print(f"\n--- Shaft {shaft} ---")

        print("Left Reaction:")
        print("  Total:  ", fmt_vec(r["left_reaction"]))
        print("  Radial: ", fmt_vec(r["left_radial"]),
              f" |mag|={r['left_radial_mag']:.2f}")
        print("  Axial:  ", fmt_vec(r["left_axial"]),
              f" |mag|={r['left_axial_mag']:.2f}")

        print("Right Reaction:")
        print("  Total:  ", fmt_vec(r["right_reaction"]))
        print("  Radial: ", fmt_vec(r["right_radial"]),
              f" |mag|={r['right_radial_mag']:.2f}")
        print("  Axial:  ", fmt_vec(r["right_axial"]),
              f" |mag|={r['right_axial_mag']:.2f}")

def print_bearing_results(results):
    print("\n" + "="*60)
    print("BEARING RESULTS")
    print("="*60)

    for shaft, data in results.items():
        print(f"\n--- Shaft {shaft} ---")

        for side in ["left_bearing_check", "right_bearing_check"]:
            b = data[side]
            label = "Left" if "left" in side else "Right"

            print(f"\n  {label} Bearing:")
            print(f"    Fr: {b['Fr']:.2f}")
            print(f"    Fa: {b['Fa']:.2f}")

            if "P0" in b:
                print(f"    P0: {b['P0']:.2f}")
                print(f"    Static SF: {b['static_sf']:.2f}")

            if "P" in b:
                print(f"    P: {b['P']:.2f}")
                print(f"    Dynamic Util: {b['dynamic_util']:.3f}")
                print(f"    L10 (rev, millions): {b['L10_rev_millions']:.2f}")

                if "L10_hours" in b:
                    print(f"    L10 (hours): {b['L10_hours']:.2f}")

if __name__ == "__main__":
    tendon_tensions = {
        "PIP_EXT": 0,
        "PIP_FLX": 124.44,
        "MCP_FLX": 218.98,
        "MCP_EXT": 0,
        "SPLAY_A": 0,
        "SPLAY_B": 192,
    }

    reactions = solve_all_bearing_reactions_from_tendon_tensions(
        tendon_tensions=tendon_tensions,
        tendon_paths=TENDON_PATH,
        shaft_bearings=shaft_bearings,
        axial_side="left",
    )

    print_shaft_reactions(reactions)

    results = solve_and_check_all_bearings(
        tendon_tensions=tendon_tensions,
        tendon_paths=TENDON_PATH,
        shaft_bearings=shaft_bearings,
        axial_side="left",
        rpm=60.0,
    )

    print_bearing_results(results)
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
