import numpy as np

from rds_finger.statics.utils import shaft_coord, unit
from rds_finger.statics.shaft_forces import build_shaft_loads_from_tendon_tensions

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
