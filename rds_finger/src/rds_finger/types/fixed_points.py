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
