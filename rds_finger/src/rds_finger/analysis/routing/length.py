from __future__ import annotations

import numpy as np
from numpy.typing import NDArray

from rds_finger.routing.types import TendonContact, TendonPathSpec
from rds_finger.routing.router import RoutedTendon


def tendon_geometric_length(rt: RoutedTendon) -> float:
    L = 0.0
    pts = rt.points
    for i in range(len(pts) - 1):
        L += float(np.linalg.norm(pts[i + 1] - pts[i]))

    for rp in rt.pulleys:
        if rp.wrap_angle and rp.wrap_angle > 0.0:
            L += float(rp.radius * rp.wrap_angle)

    return float(L)


def tendon_spool_length(model, spec: TendonPathSpec, q: NDArray[np.float64]) -> float:
    """
    Spool length contribution from fixed contacts.

    For each TendonContact with kind="fixed":
        L += sign * r * theta_j

    NOTE: we currently use contact.side as sign. Later add explicit spool_sign.
    """
    q = np.asarray(q, float).reshape(-1)

    L = 0.0
    for item in spec.items:
        if not isinstance(item, TendonContact):
            continue
        if getattr(item, "kind", "idler") != "fixed":
            continue

        pulley = model.pulleys[item.pulley]

        # pulley.shaft may be a Shaft object OR a string key
        shaft_ref = getattr(pulley, "shaft", None)
        if shaft_ref is None:
            raise ValueError(f"Pulley '{item.pulley}' has no shaft reference.")

        if isinstance(shaft_ref, str):
            # Most common in your current setup
            if shaft_ref not in model.shafts:
                raise KeyError(f"Pulley '{item.pulley}' references unknown shaft '{shaft_ref}'.")
            shaft = model.shafts[shaft_ref]
        else:
            shaft = shaft_ref

        if shaft.dof_row is None:
            raise ValueError(
                f"Fixed contact on pulley '{item.pulley}', but its shaft has dof_row=None."
            )

        j = int(shaft.dof_row)
        if j < 0 or j >= len(q):
            raise ValueError(f"Pulley '{item.pulley}' uses dof_row={j}, but q has length {len(q)}.")

        r = float(pulley.radius)
        gain = float(getattr(shaft, "dof_gain", 1.0))
        sgn = float(getattr(item, "spool_sign", 1.0))
        L += sgn * r * gain * float(q[j])

    return float(L)


def tendon_total_length(model, spec: TendonPathSpec, q: NDArray[np.float64], rt: RoutedTendon) -> float:
    return tendon_geometric_length(rt) + tendon_spool_length(model, spec, q)

def fixed_contact_sign(spec: TendonPathSpec) -> float | None:
    """
    Returns an aggregate sign hint for fixed contacts on this tendon:
      +1 or -1 if any fixed contacts exist (uses first fixed contact side),
      None if no fixed contacts exist.

    This is a weak heuristic, but it catches accidental sign flips in config.
    """
    for item in spec.items:
        if isinstance(item, TendonContact) and getattr(item, "kind", "idler") == "fixed":
            s = float(np.sign(item.side)) if item.side != 0 else 1.0
            return s
    return None