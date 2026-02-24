from __future__ import annotations

def dip_from_pip(pip: float, coupling_ratio: float) -> float:
    """Compute thetaDIP from thetaPIP."""
    return coupling_ratio * pip