"""
fixed_points.py: Fixed points object class for tendon routing purposes. Last updated: Aileen Cleary, 04/13/2026
"""

import numpy as np

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
