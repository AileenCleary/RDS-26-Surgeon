from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple, Any, Union

import numpy as np # pyright: ignore[reportMissingImports]
# *** generate pydantic models .justfile command

from components import Shaft, BearingBase, Bearing, Pulley
from tendon_types import TendonElem, TendonContact, TendonEndpoint, TendonPath
from utils import _unit, _orthonormal, _axis_equal, _angle, _capstan_ratio

"""
GLOBAL FRAME
    - x: Proximal to distal.
    - y: Left to right.
    - z: Palmar to dorsal.

Origin in the global frame (x,y)=joint_0.center(x,y) and z=joint_1.center(z)
"""

TAU = float(2.0*np.pi)
GLOBAL_X = np.array([1.0, 0.0, 0.0], dtype=float)

def build_pulley_dict(pulleys: Iterable[Pulley]) -> Dict[str, Pulley]:
    return {p.name: p for p in pulleys}

# ---------------------------------------------------------
# Tangent-point wrap (endpoint/path version)
#   - Explicitly computes tangent points for each pulley contact.
#   - Stores p.tangent_points=[tan_in, tan_out] and p.wrap_angle.
# ---------------------------------------------------------

def select_tan_point(
        side: int, 
        p1: np.ndarray, 
        p2: np.ndarray, 
        axis: np.ndarray
) -> np.ndarray:
    """Pick between two potential tangent points via vertical convention.
    
    vertical: Defined in the pulley normal plane as:
        vertical = unit(cross(global_x, axis))

    side=+1 chooses the point with the larger dot(p,vertical).
    side=-1 chooses the point with the smaller dot(p,vertical).
    """
    axis = _unit(axis)
    vertical = _unit(np.cross(GLOBAL_X, axis))
    
    p1 = np.asarray(p1, dtype=float).reshape(3)
    p2 = np.asarray(p2, dtype=float).reshape(3)

    h1 = float(np.dot(p1, vertical))
    h2 = float(np.dot(p2, vertical))

    return p1 if (h1 >= h2) == (side >= 0) else p2
    
def find_tangent_from_point(
        point: np.ndarray, 
        pulley: Pulley, 
        side: int,
) -> np.ndarray:
    """Tangent point from an external point to a circle in the plane normal to pulley.axis.
    
    Returns the chosen tangent point (3D) and appends it to pulley.tangent_points.
    """
    c = np.asarray(pulley.center, dtype=float).reshape(3)
    p = np.asarray(point, dtype=float).reshape(3)
    r = float(pulley.radius)
    e3 = _unit(pulley.axis)

    u = p - c
    d = float(np.linalg.norm(u))
    if d <= r:
        raise ValueError("Point inside/on pulley radius, no real tangent point.")

    a = (r*r) / (d*d)
    h = (r*np.sqrt(d*d - r*r)) / (d*d)
    u_perp = np.cross(e3, u)

    t1 = c + a*u + h*u_perp
    t2 = c + a*u - h*u_perp

    t = select_tan_point(
        side=side, 
        p1=t1, 
        p2=t2, 
        axis=e3)
    pulley.tangent_points.append(t)
    return t

def find_tangent_two_circles(
        pulley1: Pulley, 
        pulley2: Pulley, 
        side2: int, 
        ref_tan1: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    """Pick a consistent external tangent between two circles with the same axis direction.
    
    Returns:
        (tan_on_pulley1, tan_on_pulley2)

    Selection rule:
        - Determine 'upper/lower' on pulley1 from ref_tan1.
        - Choose the tangent whose pulley2 point matches side2 and pulley1 branch matches ref.
    """
    if not _axis_equal(pulley1.axis, pulley2.axis):
        raise ValueError("find_tangent_two_circles requires parallel axes.")
    
    axis = _unit(pulley2.axis)
    e1, e2, e3 = _orthonormal(axis)

    vertical = _unit(np.cross(GLOBAL_X, axis))
    
    c1 = np.asarray(pulley1.center, dtype=float).reshape(3)
    c2 = np.asarray(pulley2.center, dtype=float).reshape(3)
    r1 = float(pulley1.radius)
    r2 = float(pulley2.radius)

    ref_tan1 = np.asarray(ref_tan1, dtype=float).reshape(3)
    sign1 = 1 if float(ref_tan1 @ vertical) > float(c1 @ vertical) else -1

    u = c2 - c1
    u2 = np.array([float(u @ e1), float(u @ e2)], dtype=float)
    D = float(u2 @ u2)
    if D <= 1e-12:
        raise ValueError("Circle centers coincide in the normal plane.")

    tan_points: List[Tuple[np.ndarray, np.ndarray]] = [] 

    for s in (+1.0, -1.0):
        dr = r1 - s*r2
        disc = D - dr*dr
        if disc < 0.0:
            continue
        h = float(np.sqrt(disc))

        for t in (+1.0, -1.0):
            d3 = e1*u2[0] + e2*u2[1]
            perp3 = np.cross(e3, d3)
            perp2 = np.array([float(perp3 @ e1), float(perp3 @ e2)])

            v = (u2*dr + perp2*(h*t)) / D
            p1 = c1 + (e1*v[0] + e2*v[1])*r1
            p2 = c2 + (e1*v[0] + e2*v[1])*(s*r2)

            sign2 = 1 if float(p1 @ vertical) > float(c1 @ vertical) else -1
            sign3 = 1 if float(p2 @ vertical) > float(c2 @ vertical) else -1
            if (sign1 == sign2) and (sign3 == int(np.sign(side2) or 1)):
                tan_points.append((p1, p2))

    if not tan_points:
        raise ValueError("No valid tangent points between circles for given routing.")
    
    p1_best, p2_best = min(tan_points, key=lambda pp: float(np.linalg.norm(pp[1] - pp[0])))
    return p1_best, p2_best

def _wrap_angle_about_axis(
        v_in: np.ndarray, 
        v_out: np.ndarray, 
        axis: np.ndarray, 
        side: int,
) -> float:
    """Wrap angle on pulley measured around +axis.
    
    v_in and v_out point from the pulley center to incoming/outgoing tangent points.
    side selects CW or CCW branch per the tendon route sign convention.
    """
    a = _unit(axis)
    u = _unit(v_in)
    v = _unit(v_out)
    s = float(np.dot(a, np.cross(u, v)))
    c = float(np.dot(u, v))

    ccw = float(np.arctan2(s, c))
    if ccw <= 0.0:
        ccw += TAU
    
    return ccw if side >= 0 else (TAU - ccw)
    
def calculate_all_wrap_angles(
        tendons: Dict[str, TendonPath], 
        pulleys: Dict[str, Pulley]
) -> None:
    """Populate all pulley tangent points and wrap angles for endpoint-style tendon paths."""
    for p in pulleys.values():
        p._clear_runtime()
    # for name, tendon in tendons.items():
    #     print(name, [type(c).__name__ for c in tendon.contacts])

    for tendon_path in tendons.values():
        contacts = list(tendon_path.contacts)
        if len(contacts) < 3:
            continue

        for i in range(1, len(contacts) - 1):
            cur = contacts[i]
            if not isinstance(cur, TendonContact):
                continue

            prev = contacts[i - 1]
            nxt = contacts[i + 1]
            p_cur = pulleys[cur.pulley_name]
            side_cur = int(np.sign(cur.sign) or 1)

            p_cur.tangent_points = []

            if isinstance(prev, TendonEndpoint):
                tan_in = find_tangent_from_point(prev.coordinates, p_cur, side_cur)

            elif isinstance(prev, TendonContact):
                p_prev = pulleys[prev.pulley_name]
                if len(p_prev.tangent_points) < 2:
                    raise ValueError(f"{p_prev.name}: missing tangent points.")
                if not _axis_equal(p_prev.axis, p_cur.axis):
                    raise ValueError("Change of axes between pulleys not supported.")  
                ref_tan1 = p_prev.tangent_points[1]
                _, tan_in = find_tangent_two_circles(p_prev, p_cur, side_cur, ref_tan1)
    
            else:
                raise ValueError("Invalid element type.")

            if isinstance(nxt, TendonEndpoint):
                p_cur.tangent_points = [tan_in]
                tan_out = find_tangent_from_point(nxt.coordinates, p_cur, side_cur)

            elif isinstance(nxt, TendonContact):
                p_next = pulleys[nxt.pulley_name]
                side_next = int(np.sign(nxt.sign) or 1)
                if not _axis_equal(p_cur.axis, p_next.axis):
                    raise ValueError("Change of axes between pulleys not supported.")
                tan_on_cur, _ = find_tangent_two_circles(p_cur, p_next, side_next, tan_in)
                tan_out = tan_on_cur
            
            else:
                raise ValueError("Invalid element type.")
            
            p_cur.tangent_points = [tan_in, tan_out]
            p_cur.wrap_angle = _wrap_angle_about_axis(
                tan_in - p_cur.center,
                tan_out - p_cur.center,
                p_cur.axis,
                side_cur,
            )

# ---------------------------------------------------------
# Endpoint-path pulley load using stored tangent points and wrap angle.
# ---------------------------------------------------------

def compute_all_pulley_loads_endpoint_path(
        tendon: TendonPath, 
        pulleys: Dict[str, Pulley], 
        T0: float, 
        *, 
        friction_mode: str = "decay", 
        mu_new: Optional[float] = None,
) -> Tuple[List[Pulley], float]:
    """Compute pulley forces for endpoint-style tendon paths where tangent points were precomputed.
    
    Returns:
        touched: Pulleys that received loads - their force_xyz & tendon_forces updated.
        T_end: Final tension after last internal contact.
    """
    contacts = list(tendon.contacts)
    if len(contacts) < 3:
        return [], float(T0)

    fm = friction_mode.lower()
    if fm not in {"none", "decay", "growth"}:
        raise ValueError(f"Invalid entry for friction_mode: {fm}.")
    
    def point_for(
            elem: Union[TendonEndpoint, TendonContact], 
            *, 
            use_incoming: bool,
    ) -> np.ndarray:
        if isinstance(elem, TendonEndpoint):
            return np.asarray(elem.coordinates, float).reshape(3)
        p = pulleys[elem.pulley_name]
        if len(p.tangent_points) < 2:
            raise RuntimeError(
                f"{p.name}: tangent_points not initialized. "
                "calculate_all_wrap_angles() did not process this contact."
            )
        return p.tangent_points[0] if use_incoming else p.tangent_points[1]

        
    touched: List[Pulley] = []
    T_in = float(T0)

    for i in range(1, len(contacts) - 1):
        cur = contacts[i]
        if not isinstance(cur, TendonContact):
            continue

        prev = contacts[i - 1]
        nxt = contacts[i + 1]
        p_cur = pulleys[cur.pulley_name]

        mu = float(mu_new) if mu_new is not None else float(getattr(p_cur, "mu", 0.0))
        wrap = float(p_cur.wrap_angle)
        
        ratio = _capstan_ratio(mu, wrap) if fm != "none" else 1.0
        if fm == "none":
            T_out = T_in
        elif fm == "decay":
            T_out = T_in / ratio
        else:
            T_out = T_in*ratio
        
        prev_point = point_for(prev, use_incoming=False)
        next_point = point_for(nxt, use_incoming=True)

        tan_in, tan_out = p_cur.tangent_points[0], p_cur.tangent_points[1]
        t_in = _unit(prev_point - tan_in)
        t_out = _unit(next_point - tan_out)

        F_xyz = T_in*t_in + T_out*t_out
        p_cur._add_force(tendon.name, F_xyz)
        
        if p_cur not in touched:
            touched.append(p_cur)

        T_in = T_out
    
    return touched, float(T_in)

# ---------------------------------------------------------
# Vector/path wrap & tangent-direction force.
# ---------------------------------------------------------

def wrap_angle_from_path(
        c_prev : np.ndarray,
        c_cur : np.ndarray, 
        c_next : np.ndarray,
) -> float:
    """Wrap angle approximation as deflection angle of centerline path between pulleys."""
    u_in = _unit(np.asarray(c_prev, dtype=float) - np.asarray(c_cur, dtype=float))
    u_out = _unit(np.asarray(c_next, dtype=float) - np.asarray(c_cur, dtype=float))
    return _angle(u_in, u_out)

def tan_dir_at_pulley( # ***
        c_cur : np.ndarray,
        c_other : np.ndarray,
        v_hat : np.ndarray,
        r_cur : float,
        side : int,
) -> np.ndarray:
    """Tangent direction from current to neighboring pulley in plane normal to joint axis."""
    c_cur = np.asarray(c_cur, dtype=float).reshape(3)
    c_other = np.asarray(c_other, dtype=float).reshape(3)

    _, _, e3 = _orthonormal(v_hat)
    v = c_other - c_cur

    v_plane = v - float(np.dot(v, e3)) * e3
    d = float(np.linalg.norm(v_plane))
    if d <= 1e-12:
        raise ValueError("Pulley neighbor direction undefined.")
   
    u = v_plane/d
    
    r = float(r_cur) 
    if r <= 0.0 or d <= r:
        return _unit(v_plane)
    
    alpha = float(np.arcsin(np.clip(r/d, 0.0, 1.0)))
    u_perp = np.cross(e3, u)
    u_tan = np.cos(alpha)*u + side*np.sin(alpha)*u_perp
    return _unit(u_tan)

def wrap_angle_from_tangents( # ***
        c_prev : np.ndarray,
        p_cur : Pulley,
        c_next : np.ndarray,
        side_in : int,
        side_out : int,
) -> float:
    """Wrap angle calculated as angle between incoming and outgoing tangents at pulley."""

    t_in = tan_dir_at_pulley(p_cur.center, c_prev, p_cur.axis, p_cur.radius, side_in)
    t_out = tan_dir_at_pulley(p_cur.center, c_next, p_cur.axis, p_cur.radius, side_out)
    return _angle(t_in, t_out)

def pulley_force_vector( # ***
        c_prev : np.ndarray,
        c_cur : np.ndarray, 
        c_next : np.ndarray,
        T_in : float,
        T_out : float,
        ) -> np.ndarray:
    """Net force on a pulley from incoming/outgoing tendon segment tensions."""
   
    u_in = _unit(np.asarray(c_prev) - np.asarray(c_cur))
    u_out = _unit(np.asarray(c_next) - np.asarray(c_cur))
    return T_in*u_in + T_out*u_out

def force_angle_xy(F_xyz: np.ndarray) -> float: # ***
    """Angle (rads) of force projection in global XY plane."""
    
    F = np.asarray(F_xyz, dtype=float).reshape(3)
    return float(np.arctan2(F[1], F[0]))

def compute_tendon_pulley_loads( # ***
        tendon : TendonPath,
        pulleys : Dict[str, Pulley],
        T0 : float,
        *,
        friction_mode : str = "none",
        mu_new : Optional[float] = None,  
        use_tan_wrap : bool = False,
        use_tan_force : bool = False,     
) -> Tuple[List[Tuple[Pulley, np.ndarray]], float]:
    """
    Compute per pulley forces along tendon path with friction options.
    
    Only internal pulleys, excluding endpoints, have a computed load (requires >=3 contacts).
    friction_mode:
        - "none" : T_out = T_in (ideal idlers, no friction)
        - "decay" : T_out = T_in / exp(mu*wrap) (friction loss)
        - "growth" : T_out = T_in * exp(mu*wrap) (conservative friction loss, worst-case)
    """ 
    contacts = list(tendon.contacts)
    if len(contacts) < 3:
        return [], float(T0)


    if any(not isinstance(c, TendonContact) for c in contacts):
        raise ValueError("compute_tendon_pulley_loads only accounts for contact-only TendonPath.")
    
    fm = friction_mode.lower()
    if fm not in {"none", "decay", "growth"}:
        raise ValueError(f"Invalid entry for friction_mode: {fm}.")
    
    forces: List[Tuple[Pulley, np.ndarray]] = []
    T_in = float(T0)

    for i in range(1, len(contacts) - 1):
        prev = contacts[i - 1]
        cur = contacts[i]
        nxt = contacts[i + 1]

        p_prev = pulleys[prev.pulley_name] # type: ignore
        p_cur = pulleys[cur.pulley_name] # type: ignore
        p_next = pulleys[nxt.pulley_name] # type: ignore

        side = int(np.sign(cur.sign) or 1) # type: ignore

        wrap = (
            wrap_angle_from_tangents(p_prev.center, p_cur, p_next.center, side, side)
            if use_tan_wrap
            else wrap_angle_from_path(p_prev.center, p_cur.center, p_next.center)
        )

        mu = float(mu_new) if mu_new is not None else float(getattr(p_cur, "mu", 0.0))
        ratio = _capstan_ratio(mu, wrap) if fm != "none" else 1.0

        if fm == "none":
            T_out = T_in
        elif fm == "decay":
            T_out = T_in/ratio
        else:
            T_out = T_in*ratio

        if use_tan_force:
            t_in = tan_dir_at_pulley(p_cur.center, p_prev.center, p_cur.axis, p_cur.radius, side)
            t_out = tan_dir_at_pulley(p_cur.center, p_next.center, p_cur.axis, p_cur.radius, side)
            F_xyz = T_in*t_in + T_out*t_out
        else:
            F_xyz = pulley_force_vector(p_prev.center, p_cur.center, p_next.center, T_in, T_out)
        
        forces.append((p_cur, F_xyz))
        T_in = T_out

    return forces, float(T_in)

# ---------------------------------------------------------
# Statics & bearing selection functions.
# ---------------------------------------------------------

def bending_stress_max(M_Nmm: float, d_mm: float) -> float:
    """Max bending stress (MPa) on a solid circular shaft."""

    return 32*float(M_Nmm)/(np.pi*float(d_mm)**3)

def torsion_shear_stress_max(T_Nmm: float, d_mm: float) -> float:
    """ Max torsional shear stress (Mpa) on a solid circular shaft."""

    return 16*float(T_Nmm)/(np.pi*float(d_mm)**3)

def von_mises_bending_torsion(sigma_MPa: float, tau_MPa: float) -> float:
    """Von Mises for combined bending (sigma) and torsion (tau)."""

    s = float(sigma_MPa)
    t = float(tau_MPa)
    return float(np.sqrt(s*s + 3.0*t*t))

def bearing_equivalent_load(
        Fr: float,
        Fa: float,
        bearing: BearingBase,
        *,
        X: Optional[float] = None,
        Y: Optional[float] = None,
) -> float:
    """Equivalent dynamic bearing load:
        P = X*Fr + Y*Fa
    
    X, Y: Depend on the bearing type and Fa/Fr from bearing catalogue.
    Either pass in explicitly or use default from bearing base."""

    x = float(X) if X is not None else float(getattr(bearing, "X", 1.0))
    y = float(Y) if Y is not None else float(getattr(bearing, "Y", 0.0))
    return x*float(Fr) +  y*float(Fa)

def L10_revolutions(
        C: float, 
        P: float, 
        p: float = 3.0
) -> float:
        """Basic bearing life (revolutions).
        
        p: 3.0 for ball bearings."""

        return (float(C)/float(P))**float(p)*1e6

def pulley_s_mm(
        p: Pulley,
        bearing1_center: np.ndarray,
) -> float:
    """Axial coordinate along shaft axis from bearing1 to pulley center.
    
    s=0 at bearing1 : s+ -> +axis
    """
    a = _unit(p.axis)
    return float(np.dot(np.asarray(p.center, float) - np.asarray(bearing1_center, float), a))

class ShaftAnalysis:
    """ Simply supported shaft statics using point loads in 3D."""

    @staticmethod
    def bearing_reactions_3d(
        loads : List[Pulley],
        bearing1_pos : np.ndarray,
        bearing2_pos : np.ndarray,
        axis : np.ndarray,
        use_pulley_s: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Compute bearing reactions R1 & R2 (global vectors) for a simply supported shaft."""

        b1 = np.asarray(bearing1_pos, dtype=float).reshape(3)
        b2 = np.asarray(bearing2_pos, dtype=float).reshape(3)
        e1, e2, e3 = _orthonormal(axis)

        L_b = float(np.dot(b2 - b1, e3))
        if abs(L_b) <= 1e-12:
            raise ValueError("Bearing span along axis is near zero.")
        
        sumF = np.zeros(2, dtype=float)
        sumFs = np.zeros(2, dtype=float)

        for p in loads:
            F = np.asarray(getattr(p, "force_xyz", np.zeros(3)), dtype=float).reshape(3)
            s = float(getattr(p, "s_mm", 0.0))
            if not use_pulley_s:
                s = pulley_s_mm(p, b1)
                p.s_mm = s
            
            f1 = float(np.dot(F, e1))
            f2 = float(np.dot(F, e2))

            sumF += np.array([f1, f2])
            sumFs += np.array([f1*s, f2*s]) # type: ignore

        R2_12 = sumFs / L_b
        R1_12 = sumF - R2_12
        
        R1_xyz = R1_12[0]*e1 + R1_12[1]*e2
        R2_xyz = R2_12[0]*e1 + R2_12[1]*e2
        
        return R1_xyz, R2_xyz, L_b

def going_insane(
    tendon: TendonPath,
    pulleys: Dict[str, Pulley],
    *,
    T0: float = 50.0,
    mu: float = 0.05,
    friction_mode: str = "decay",
    use_tan_wrap: bool = True,
    use_tan_force: bool = True,
) -> None:
    """Try and check my work so far somewhat."""

    for p in pulleys.values():
        p.mu = mu

    forces, T_end = compute_tendon_pulley_loads(
        tendon, pulleys, T0,
        friction_mode=friction_mode,
        mu_new=mu,
        use_tan_wrap=use_tan_wrap,
        use_tan_force=use_tan_force
    )

    print(f"\nTendon: {tendon.name}")
    print(f"    contacts: {len(tendon.contacts)} (>=3 for internal loads.)")
    print(f"    T0={T0:.2f} N -> Tend={T_end:.2f} N (mode={friction_mode}, mu={mu:.3f})")

    for p, F in forces:
        mag = float(np.linalg.norm(F))
        ang_xy = np.degrees(np.arctan2(F[1], F[0]))
        ang_xz = np.degrees(np.arctan2(F[2], F[0]))
        print(f"\n{p.name:10s} \ncenter={p.center} |F|={mag:8.2f} N") 
        print(f"angle_xy={ang_xy:8.2f} degrees, angle_xz={ang_xz:8.2f} degrees.")
