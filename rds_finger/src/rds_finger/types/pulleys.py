"""
pulleys.py: Pulley object class for tendon routing and force analysis. Last updated: Aileen Cleary, 04/13/2026
"""

import numpy as np

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
