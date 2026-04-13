import numpy as np

from rds_finger.config import SHAFT_LOADS
from rds_finger.types.pulleys import Pulley
from rds_finger.statics.pulley_forces import PulleyLoad
from rds_finger.statics.tangent import (
    force_dir_on_pulley_from_prev,
    force_dir_on_pulley_to_next,
)


def segment_tensions_from_value(
        tendon_path,
        tension_value,
):
    nseg = len(tendon_path) - 1

    if np.isscalar(tension_value):
        return [float(tension_value)] * nseg

    vals = list(tension_value)
    if len(vals) != nseg:
        raise ValueError(f"Expected {nseg} segment tensions, got {len(vals)}.")
    return vals


def build_shaft_loads_from_tendon_tensions(
        tendon_tensions,
        tendon_paths,
):
    shaft_loads = {shaft: [] for shaft in SHAFT_LOADS.keys()}

    for tendon_name, tendon_path in tendon_paths.items():
        if tendon_name not in tendon_tensions:
            continue

        tensions = segment_tensions_from_value(
            tendon_path=tendon_path,
            tension_value=tendon_tensions[tendon_name],
        )

        for i in range(1, len(tendon_path) - 1):
            prev = tendon_path[i - 1]
            cur = tendon_path[i]
            nxt = tendon_path[i + 1]

            if not isinstance(cur, Pulley):
                continue

            v_in = force_dir_on_pulley_from_prev(prev, cur)
            v_out = force_dir_on_pulley_to_next(cur, nxt)

            t_in = tensions[i - 1]
            t_out = tensions[i]

            shaft_loads[cur.shaft].append(
                PulleyLoad(cur, v_in, v_out, t_in, t_out)
            )

    return shaft_loads