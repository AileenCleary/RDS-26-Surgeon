import numpy as np

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
