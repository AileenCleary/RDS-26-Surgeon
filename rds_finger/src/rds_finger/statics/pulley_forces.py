import numpy as np
from rds_finger.statics.utils import unit
from rds_finger.statics.tangent import (
    force_dir_on_pulley_from_prev,
    force_dir_on_pulley_to_next,
)
from rds_finger.types.pulleys import Pulley
from rds_finger.types.fixed_points import Point3D
from rds_finger.config import SHAFT_LOADS

class PulleyLoad:
    def __init__(
            self,
            pulley: Pulley,
            v_in,
            v_out,
            t_in=0.0,
            t_out=0.0,
    ):
        self.pulley = pulley
        self.center = pulley.center.copy()
        self.axis = pulley.axis.copy()
        self.shaft = pulley.shaft

        self.v_in = unit(v_in)
        self.v_out = unit(v_out)
        self.t_in = t_in
        self.t_out = t_out

        self.f_in = self.t_in * self.v_in
        self.f_out = self.t_out * self.v_out
        self.f_net = self.f_in + self.f_out

    def set_tensions(
            self,
            t_in,
            t_out,
    ):
        self.t_in = t_in
        self.t_out = t_out
        self.f_in = self.t_in * self.v_in
        self.f_out = self.t_out * self.v_out
        self.f_net = self.f_in + self.f_out

def init_tendon_pulley_loads(
        tendon_path,
        tensions=None,
):
    start = tendon_path[0]
    end = tendon_path[-1]

    if not isinstance(start, Point3D) or not isinstance(end, Point3D):
        raise ValueError("Tendon paths need to start and end with a Point.")
    if (start.type != "START") or (end.type != "END"):
        raise ValueError("Start and end points must be correctly labelled.")

    if tensions is None:
        tensions = [0.0] * (len(tendon_path) - 1)

    for key in SHAFT_LOADS:
        SHAFT_LOADS[key] = []

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

        SHAFT_LOADS[cur.shaft].append(
            PulleyLoad(cur, v_in, v_out, t_in, t_out)
        )
