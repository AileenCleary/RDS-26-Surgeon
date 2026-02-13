from __future__ import annotations

from dataclasses import dataclass
from math import tau
from typing import Dict, Iterable, List, Optional, Tuple

import numpy as np # pyright: ignore[reportMissingImports]
# *** generate pydantic models .justfile command

from components import Shaft, BearingBase

"""
GLOBAL FRAME
    - x: Proximal to distal.
    - y: Left to right.
    - z: Palmar to dorsal.

Origin in the global frame (x,y)=joint_0.center(x,y) and z=joint_1.center(z)
"""

class pulley:
    "Pulley variable base dimensions (mm)."
    def __init__(self, radius: float, width: float):
        self.radius = radius
        self.width = width

class PulleyPose(pulley):
    """A specific pulley instance mounted on a shaft in the global frame.
    
    tendon: Tendons are assigned a constant y-offset (parallel to global x axis)"""

    def __init__(
        self, 
        base : pulley,
        name: str, 
        shaft: Shaft,
        tendon_y_axes: List[float],
        *,
        tendon: int = 0,
        mu: float = 0.0,
    ):
        
        super().__init__(base.radius, base.width)
        self.name = name
        self.mu = mu
        self.tendon = tendon
        self.shaft = shaft

        a = np.asarray(self.shaft.axis, dtype=float).reshape(3)
        na = float(np.linalg.norm(a))
        if na <= 1e-12:
            raise ValueError("INVALID: Shaft axis is nonzero.")
        self.axis = a/ na

        lane_y = tendon_y_axes[self.tendon]
        c = np.asarray(self.shaft.center, dtype=float).reshape(3)
        self.center = c + lane_y * np.array(self.shaft.axis, dtype=float)

@dataclass(frozen=True)
class TendonContact:
    """Tendon contact on a pulley.
    
    side: Determines which geometric tangent is used at that pulley in the normal plane of the shaft axis.
    (Hint) The convention I used is in config.py: 
        Point thumb in positive direction of joint axis.
        Imagine pulling on the tendon. Resulting rotation of pulley determines sign,
        i.e., (+CCW,-CW) relative to the positive joint axis."""
    
    pulley_name: str
    side: int

@dataclass(frozen=True)
class TendonPath:
    """Ordered pulley contacts along a tendon path (proximal to distal)."""
    
    name: str
    contacts: List[TendonContact]

@dataclass(frozen=True)
class PulleyLoad:
    """
    Point load applied at a pulley.

    force_xyz : Net force on pulley in global frame (N).
    s_mm : Axial coordinate along shaft axis from bearing1 (mm).
    """

    pulley: PulleyPose
    force_xyz: np.ndarray
    s_mm: float


def _unit(v : np.ndarray) -> np.ndarray:
    v = np.asarray(v, dtype=float).reshape(3)
    n = float(np.linalg.norm(v))
    if n <= 1e-12:
        raise ValueError("Vector too close to zero.")
    return v / n

def _angle(u : np.ndarray, v : np.ndarray) -> float:
    """Angle between vectors (rads), in range [0,pi]."""

    u = _unit(u)
    v = _unit(v)
    c = float(np.clip(np.dot(u,v), -1.0, 1.0))
    return float(np.arccos(c))

def _orthonormal(axis_hat : np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build right-hand orthonormal basis (e1,e2,e3) where e3 is along axis hat."""
    
    e3 = _unit(axis_hat)
    temp = np.array([1.0, 0.0, 0.0], dtype=float) # global x axis
    if abs(float(np.dot(temp, e3))) > 0.9:
        temp = np.array([0.0, 1.0, 0.0])
    e1 = _unit(np.cross(e3, temp))
    e2 = np.cross(e3, e1)
    return e1, e2, e3

def capstan_ratio(mu : float, wrap_angle_rad : float) -> float:
    """Capstan equation ratio."""
    
    return float(np.exp(mu * wrap_angle_rad)) 

def wrap_angle_from_path(
        c_prev : np.ndarray,
        c_cur : np.ndarray, 
        c_next : np.ndarray,
        ) -> float:
    """Wrap angle approximation as deflection angle of centerline path between pulleys."""
    
    u_in = _unit(np.asarray(c_prev) - np.asarray(c_cur))
    u_out = _unit(np.asarray(c_next) - np.asarray(c_cur))
    return _angle(u_in, u_out)

def tan_dir_at_pulley(
        c_cur : np.ndarray,
        c_other : np.ndarray,
        v_hat : np.ndarray,
        r_cur : float,
        side : int,
) -> np.ndarray:
    """Tangent direction from current to neighboring pulley in plane normal to joint axis."""
    
    c_cur = np.asarray(c_cur, float).reshape(3)
    c_other = np.asarray(c_other, float).reshape(3)

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

def wrap_angle_from_tangents(
        c_prev : np.ndarray,
        p_cur : PulleyPose,
        c_next : np.ndarray,
        side_in : int,
        side_out : int,
) -> float:
    """Wrap angle calculated as angle between incoming and outgoing tangents at pulley."""

    t_in = tan_dir_at_pulley(p_cur.center, c_prev, p_cur.axis, p_cur.radius, side_in)
    t_out = tan_dir_at_pulley(p_cur.center, c_next, p_cur.axis, p_cur.radius, side_out)
    return _angle(t_in, t_out)

def pulley_force_vector(
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

def force_angle_xy(F_xyz: np.ndarray) -> float:
    """Angle (rads) of force projection in global XY plane."""
    
    F = np.asarray(F_xyz, dtype=float).reshape(3)
    return float(np.arctan2(F[1], F[0]))

def compute_tendon_pulley_loads(
        tendon : TendonPath,
        pulleys : Dict[str, PulleyPose],
        T0 : float,
        *,
        friction_mode : str = "none",
        mu_new : Optional[float] = None,  
        use_tan_wrap : bool = False,
        use_tan_force : bool = False,     
) -> Tuple[List[Tuple[PulleyPose, np.ndarray]], float]:
    """
    Compute per pulley forces along tendon path with friction options.
    
    Only internal pulleys, excluding endpoints, have a computed load (requires >=3 contacts).
    friction_mode:
        - "none" : T_out = T_in (ideal idlers, no friction)
        - "decay" : T_out = T_in / exp(mu*wrap) (friction loss)
        - "growth" : T_out = T_in * exp(mu*wrap) (conservative friction loss, worst-case)
    """ 

    contacts = tendon.contacts
    if len(contacts) < 3:
        return [], float(T0)

    fm = friction_mode.lower()
    if fm not in {"none", "decay", "growth"}:
        raise ValueError("friction_mode input invalid.")
    
    forces: List[Tuple[PulleyPose, np.ndarray]] = []
    T_in = float(T0)

    for i in range(1, len(contacts) - 1):
        p_prev = pulleys[contacts[i-1].pulley_name]
        p_cur = pulleys[contacts[i].pulley_name]
        p_next = pulleys[contacts[i+1].pulley_name]

        side = 1 if contacts[i].side >= 0 else -1

        wrap = (
            wrap_angle_from_tangents(p_prev.center, p_cur, p_next.center, side, side)
            if use_tan_wrap
            else wrap_angle_from_path(p_prev.center, p_cur.center, p_next.center)
        )

        mu = float(mu_new) if mu_new is not None else float(p_cur.mu)
        ratio = capstan_ratio(mu, wrap) if fm != "none" else 1.0

        if friction_mode == "none":
            T_out = T_in
        elif friction_mode == "decay":
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
    """Equivalent dynamic bearing load.
    
    X, Y: Depend on the bearing type and Fa/Fr from bearing catalogue.
    Either pass in explicitly or use default from bearing base."""

    x = X if X is not None else getattr(bearing, "X", 1.0)
    y = Y if Y is not None else getattr(bearing, "Y", 0.0)
    return float(x)*float(Fr) + float(y)*float(Fa)

def L10_revolutions(C: float, P: float, p: float = 3.0):
        """Basic bearing life (revolutions).
        
        p: 3.0 for ball bearings."""

        return (float(C)/float(P))**float(p)*1e6

class ShaftAnalysis:
    """ Simply supported shaft statics using point loads."""

    @staticmethod
    def load_s_mm(pulley_center: np.ndarray, bearing1_pos: np.ndarray, axis_hat: np.ndarray) -> float:
        """Axial coordinate s along shaft axis from bearing 1 to pulley center."""
        e3 = _unit(axis_hat)
        return float(np.dot(np.asarray(pulley_center) - np.asarray(bearing1_pos), e3))

    @staticmethod
    def bearing_reactions_3d(
        loads : List[PulleyLoad],
        bearing1_pos : np.ndarray,
        bearing2_pos : np.ndarray,
        v_hat : np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, List[PulleyLoad], float]:
        """Compute bearing reactions R1 & R2 (global vectors) for a simply supported shaft."""

        b1 = np.asarray(bearing1_pos, dtype=float).reshape(3)
        b2 = np.asarray(bearing2_pos, dtype=float).reshape(3)
        e1, e2, e3 = _orthonormal(v_hat)

        L_b = float(np.dot(b2 - b1, e3))
        if abs(L_b) <= 1e-12:
            raise ValueError("INVALID: Bearing span along axis is near zero.")
        
        sumF = np.zeros(2, dtype=float)
        sumFs = np.zeros(2, dtype=float)
        loads_out: List[PulleyLoad] = []

        for ld in loads:
            s = ShaftAnalysis.load_s_mm(ld.pulley.center, b1, e3)
            F = np.asarray(ld.force_xyz, dtype=float).reshape(3)
            
            f1 = float(np.dot(F, e1))
            f2 = float(np.dot(F, e2))

            sumF += np.array([f1, f2])
            sumFs += np.array([f1*s, f2*s])
            
            loads_out.append(PulleyLoad(pulley=ld.pulley, force_xyz=F, s_mm=s))

        R2_12 = sumFs/L_b
        R1_12 = sumF - R2_12
        
        R1_xyz = R1_12[0]*e1 + R1_12[1]*e2
        R2_xyz = R2_12[0]*e1 + R2_12[1]*e2
        
        return R1_xyz, R2_xyz, loads_out, L_b



def build_pulley_dict(pulleys: Iterable[PulleyPose]) -> Dict[str, PulleyPose]:
    return {p.name: p for p in pulleys}

def going_insane(
    tendon: TendonPath,
    pulleys: Dict[str, PulleyPose],
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
        #ang = np.degrees(force_angle_xy(F))
        ang_xy = np.degrees(np.arctan2(F[1], F[0]))
        ang_xz = np.degrees(np.arctan2(F[2], F[0]))
        print(f"\n{p.name:10s} \ncenter={p.center} |F|={mag:8.2f} N") 
        print(f"angle_xy={ang_xy:8.2f} degrees, angle_xz={ang_xz:8.2f} degrees.")
