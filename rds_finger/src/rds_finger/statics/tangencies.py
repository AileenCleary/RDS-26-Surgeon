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
    ) -> None:
        self.center = np.asarray(center, dtype=float).reshape(3)
        self.axis = np.asarray(axis, dtype=float).reshape(3)
        self.shaft = shaft
        self.side = side
        self.c0 = c0
        self.c = c

        if name is not None:
            self.name = name


class ShaftPulleyForce:
    def __init__(
            self,
            pulley,
            f_in,
            f_out,
    ):
        self.pulley = pulley
        self.f_in = np.asarray(f_in, dtype=float).reshape(3)
        self.f_out = np.asarray(f_out, dtype=float).reshape(3)
        self.f_net = self.f_in + self.f_out
        self.center = pulley.center.copy()
        self.shaft = pulley.shaft


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


def tangent_point_3d_point_to_pulley(
        point,
        pulley,
):
    ex, ey, n = plane_basis_with_vertical(pulley.axis)
    t2 = choose_tangent_point_to_pulley(point, pulley)
    return lift_from_plane_coords(t2, pulley.center, ex, ey)


def tangent_points_3d_pulley_to_pulley(
        pulley1,
        pulley2,
):
    axis = pulley1.axis
    ex, ey, n = plane_basis_with_vertical(axis)
    pts2 = choose_tangent_pulley_to_pulley(pulley1, pulley2)
    t1 = lift_from_plane_coords(pts2[0], pulley1.center, ex, ey)
    t2 = lift_from_plane_coords(pts2[1], pulley1.center, ex, ey)
    return t1, t2


def segment_force_on_pulley(
        tangent_point,
        other_point,
        tension,
):
    u = unit(np.asarray(other_point, dtype=float) - np.asarray(tangent_point, dtype=float))
    return tension * u


def force_from_prev_to_pulley(
        prev,
        pulley,
        tension,
):
    if isinstance(prev, Point3D):
        t = tangent_point_3d_point_to_pulley(prev, pulley)
        return segment_force_on_pulley(t, prev.center, tension)

    t_prev, t_cur = tangent_points_3d_pulley_to_pulley(prev, pulley)
    return segment_force_on_pulley(t_cur, t_prev, tension)


def force_from_pulley_to_next(
        pulley,
        nxt,
        tension,
):
    if isinstance(nxt, Point3D):
        t = tangent_point_3d_point_to_pulley(nxt, pulley)
        return segment_force_on_pulley(t, nxt.center, tension)

    t_cur, t_next = tangent_points_3d_pulley_to_pulley(pulley, nxt)
    return segment_force_on_pulley(t_cur, t_next, tension)


def build_shaft_pulley_forces(
        tendon_path,
        tensions,
):
    pulley_forces = []

    for i in range(1, len(tendon_path) - 1):
        cur = tendon_path[i]

        if not isinstance(cur, Pulley):
            continue

        prev = tendon_path[i - 1]
        nxt = tendon_path[i + 1]

        f_in = force_from_prev_to_pulley(prev, cur, tensions[i - 1])
        f_out = force_from_pulley_to_next(cur, nxt, tensions[i])

        pulley_forces.append(ShaftPulleyForce(cur, f_in, f_out))

    return pulley_forces


def group_pulley_forces_by_shaft(
        pulley_forces,
):
    out = {}

    for load in pulley_forces:
        if load.shaft not in out:
            out[load.shaft] = []
        out[load.shaft].append(load)

    return out


def solve_bearing_reactions_for_shaft(
        left_bearing,
        right_bearing,
        pulley_forces,
        axial_side="left",
):
    axis = unit(left_bearing.axis)

    L = shaft_coord(right_bearing.center, left_bearing.center, axis)

    sum_f_rad = np.zeros(3)
    sum_m_left = np.zeros(3)
    sum_f_ax = 0.0

    for load in pulley_forces:
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