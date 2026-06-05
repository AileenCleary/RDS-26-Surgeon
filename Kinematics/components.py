from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple, List
import numpy as np # pyright: ignore[reportMissingImports]

from utils import _unit
@dataclass(frozen=True)
class Motor:
    Pmax: float
    Vmax: float
    eff: float
    T: float
@dataclass(frozen=True)
class Material:
    Sy: float
    Sut: float

    @property
    def T_allow(self) -> float:
        return 0.577 * float(self.Sy)
    
class ShaftBase:
    def __init__(
            self,
            diameter: float,
            length: float,
            axis: np.ndarray,
            material: Optional[Material] = None
    ):
        self.diameter = diameter
        self.length = length
        self.axis = axis
        if material is not None:
            self.material = material

class Shaft(ShaftBase):
    def __init__(
            self,
            base: ShaftBase,
            name: str,
            center: np.ndarray,
            dof_row: int | None = None,
            alias: str | None = None,
    ):
        super().__init__(
            base.diameter,
            base.length,
            base.axis
        )

        self.name = name
        self.center = center
        self.dof_row = dof_row
        self.alias = alias
        
class BearingBase:
    def __init__(self,
                inner_diameter: float,
                outer_diameter: float,
                shaft_diameter: float,
                width: float,
                X: float = 1.0,
                Y: float = 0.0,
                p: float = 3.0,
                C: float = 0.0,
                C0: float = 0.0,
                ):
        self.inner_diameter = inner_diameter
        self.outer_diameter = outer_diameter
        self.shaft_diameter = shaft_diameter
        self.width = width
        self.X = X
        self.Y = Y
        self.p = p
        self.C = C
        self.C0 = C0
class Bearing(BearingBase):
    def __init__(self,
                base : BearingBase,
                shaft : Shaft,
                side: int,
                alias : Optional[str] = None,
                ):
    
        super().__init__(
            base.inner_diameter,
            base.outer_diameter,
            base.shaft_diameter,
            base.width,
            base.X,
            base.Y,
            base.p,
            base.C,
            base.C0
        )
        self.shaft = shaft
        self.alias = alias
        
        c = np.asarray(self.shaft.center, dtype=float).reshape(3)
        a = np.asarray(self.shaft.axis, dtype=float).reshape(3)
        na = float(np.linalg.norm(a))
        a = a / na

        sign = 1.0 if side >= 0 else -1.0
        d = 0.5*self.shaft.length - 0.5*self.width
        self.center = (c + sign*d*a).tolist()
class PulleyBase:
    def __init__(
            self,
            radius: float,
            width: float
    ):
        self.radius = radius
        self.width = width
class Pulley(PulleyBase):
    def __init__(
            self,
            base: PulleyBase,
            name: str,
            lane: float,
            shaft: Shaft, # *** "Shaft"?
            role: str,
            # tendon_id: int,
            mu: float = 0.0,
            shaft_name: Optional[str] = None,
            tangent_points: Optional[List[np.ndarray]] = None,
            # tendon_name: Optional[str] = "",
    ):  
        super().__init__(
            base.radius,
            base.width
        )
        self.role = role
        self.name = name
        self.shaft = shaft
        self.lane = lane
        self.shaft_name = shaft_name
        self.mu = float(mu)

        self.axis = _unit(self.shaft.axis)
        self.center = self.shaft.center + lane*self.axis
        self.tangent_points = list(tangent_points) if tangent_points is not None else []


        self.wrap_angle = 0.0
        self.tendon_forces = []
        self.force_xyz = np.zeros(3)
        self.s_mm = 0.0

    def _clear_runtime(self) -> None:
        """Clear per-run results, i.e., wrap/tangent points/forces."""
        self.tangent_points.clear()
        self.wrap_angle = 0.0
        self.force_xyz[:] = 0.0
        self.tendon_forces.clear()

    def _add_force(
            self,
            tendon_name: str,
            F_xyz: np.ndarray,
    ) -> None:
        """Accumulate tendon contribution to pulley load."""
        F_xyz = np.asarray(F_xyz, float).reshape(3)
        self.tendon_forces.append((tendon_name, F_xyz))
        self.force_xyz += F_xyz

@dataclass(frozen=True)
class MotorDrum:
    name: str
    radius: float
    center: np.ndarray
    axis: np.ndarray
    tendon_name: str
    direction: float = 1.0
        
