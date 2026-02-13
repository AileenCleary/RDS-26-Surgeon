from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Iterable, Optional, Tuple
import numpy as np # pyright: ignore[reportMissingImports]

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
    
@dataclass(frozen=True)
class Shaft:
    diameter: float
    length: float
    center: np.ndarray
    axis: np.ndarray
    material: Optional[Material] = None
    alias : Optional[str] = None

class BearingBase:
    def __init__(self,
                inner_diameter: float,
                outer_diameter: float,
                shaft_diameter: float,
                width: float,
                X: float = 1.0,
                Y: float = 0.0,
                p: float = 3.0,
                C: float = 0.0
                ):
        self.inner_diameter = inner_diameter
        self.outer_diameter = outer_diameter
        self.shaft_diameter = shaft_diameter
        self.width = width
        self.X = X
        self.Y = Y
        self.p = p
        self.C = C

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
            base.width
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

@dataclass(frozen=True)
class Pulley:
    R: float
    shaft: int
    pid: int
    wrap: Optional[float] = None

class Pulleys:
    def __init__(self, pulleys: Iterable[Pulley]):
        self._p: Dict[Tuple[int, int], Pulley] = {}
        for p in pulleys:
            key = (int(p.shaft), int(p.pid))
            if key in self._p:
                raise ValueError(f"Duplicate pulley key {key}.")
            self._p[key] = p
    
    def r(self, shaft: int, pid: int) -> float:
        return float(self._p[(shaft, pid)].R)
    
    def phi(self, shaft: int, pid: int) -> Optional[float]:
        return self._p[(shaft, pid)].wrap
    
    def has(self, shaft : int, pid : int) -> bool:
        return (int(shaft), int(pid)) in self._p