from __future__ import annotations
import numpy as np
from rds_finger.routing.types import (
    ShaftSpec, PulleySpec, AnchoredPoint, MotorDrumSpec, TendonContact, TendonPathSpec, BearingSpec
)

LINK_LENGTHS = dict(
    splay=33.419,
    proximal=46.00,
    middle=38.500,
    distal=23.09,
)
FINGERTIP_OFFSET = 8.624
COUPLING_RATIO = 0.7

SHAFTS: dict[str, ShaftSpec] = {
    "joint_Z": ShaftSpec(
        name="joint_Z", diameter=4.0, length=41.00,
        axis_local=np.array([0,0,1], float),
        anchor_frame="O0",
        center_local=np.array([-22.0, 0.0, -1.40], float),
        dof_row=None,
    ),
    "joint_0": ShaftSpec(
        name="joint_0", diameter=4.0, length=37.800,
        axis_local=np.array([0,0,1], float),
        anchor_frame="O0",
        center_local=np.array([0.0,0.0,0.0], float),
        dof_row=0,
    ),
    "joint_A": ShaftSpec(
        name="joint_A", diameter=4.0, length=29.00,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O0",
        center_local=np.array([15.678,0.0,0.0], float),
        dof_row=None,
    ),
    "joint_1": ShaftSpec(
        name="joint_1", diameter=4.0, length=31.80,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O1",
        center_local=np.array([0.0,0.0,0.0], float),
        dof_row=1,
    ),
    "joint_B": ShaftSpec(
        name="joint_B", diameter=4.0, length=19.00,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O1",
        center_local=np.array([LINK_LENGTHS["proximal"]*0.39, 1.5, 0.0], float),
        dof_row=None,
    ),
    "joint_2": ShaftSpec(
        name="joint_2", diameter=4.0, length=25.30,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O2",
        center_local=np.array([0.0, -0.25, 0.0], float),
        dof_row=2,
    ),
    "joint_C": ShaftSpec(
        name="joint_C", diameter=4.0, length=16.00,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O2",
        center_local=np.array([LINK_LENGTHS["middle"]*0.58, 0.0, 0.0], float),
        dof_row=None,
    ),
    "joint_3": ShaftSpec(
        name="joint_3", diameter=4.0, length=18.00,
        axis_local=np.array([0,1,0], float),
        anchor_frame="O3",
        center_local=np.array([0.0, -0.006, 0.0], float),
        dof_row=2,  # DIP not a DOF (coupled)
        dof_gain=COUPLING_RATIO
    ),
}
# -------------------------------------------------
# Bearings (NEW)
# -------------------------------------------------

def _bearing_lane_from_ll(shaft_name: str, *, side: int, ll: float) -> float:
    """
    Convert your old ll convention into a lane (mm) along the shaft axis.

    - shaft length is in SHAFTS[shaft_name].length
    - side is -1 or +1
    - ll in [0,1] is fraction of half-length from shaft center toward that side
    """
    if side not in (-1, +1):
        raise ValueError("side must be -1 or +1")
    L = float(SHAFTS[shaft_name].length)
    return float(side) * float(ll) * (0.5 * L)

# You can tune ll per bearing if you want them closer to center/edge.
BEARINGS = {
    # SPLAY shaft (joint_0) bearings
    "bearing_00": BearingSpec(name="bearing_00", shaft="joint_0", lane=_bearing_lane_from_ll("joint_0", side=-1, ll=0.8)),
    "bearing_01": BearingSpec(name="bearing_01", shaft="joint_0", lane=_bearing_lane_from_ll("joint_0", side=+1, ll=0.8)),

    # MCP shaft (joint_1) bearings
    "bearing_10": BearingSpec(name="bearing_10", shaft="joint_1", lane=_bearing_lane_from_ll("joint_1", side=-1, ll=0.8)),
    "bearing_11": BearingSpec(name="bearing_11", shaft="joint_1", lane=_bearing_lane_from_ll("joint_1", side=+1, ll=0.8)),

    # PIP shaft (joint_2) bearings
    "bearing_20": BearingSpec(name="bearing_20", shaft="joint_2", lane=_bearing_lane_from_ll("joint_2", side=-1, ll=0.8)),
    "bearing_21": BearingSpec(name="bearing_21", shaft="joint_2", lane=_bearing_lane_from_ll("joint_2", side=+1, ll=0.8)),

    # If you later model DIP as a real shaft DOF, add joint_3 bearings here.
    # For now, keep DIP bearings out unless you actually use that shaft in shaft-load aggregation.
}
PULLEYS: dict[str, PulleySpec] = {
    "pulley_Z1": PulleySpec("pulley_Z1", radius=8.0, width=3.0, shaft="joint_Z", lane=-8.5),
    "pulley_Z2": PulleySpec("pulley_Z2", radius=8.0, width=3.0, shaft="joint_Z", lane=-5.5),
    "pulley_Z3": PulleySpec("pulley_Z3", radius=8.0, width=3.0, shaft="joint_Z", lane=+5.5),
    "pulley_Z4": PulleySpec("pulley_Z4", radius=8.0, width=3.0, shaft="joint_Z", lane=+8.5),

    "pulley_01": PulleySpec("pulley_01", radius=6.650, width=3.0, shaft="joint_0", lane=-8.5),
    "pulley_02": PulleySpec("pulley_02", radius=2.75, width=3.0, shaft="joint_0", lane=-5.5),
    "pulley_03": PulleySpec("pulley_03", radius=2.75, width=3.0, shaft="joint_0", lane=+5.5),
    "pulley_04": PulleySpec("pulley_04", radius=6.650, width=3.0, shaft="joint_0", lane=+8.5),

    "pulley_A1": PulleySpec("pulley_A1", radius=8.0, width=3.0, shaft="joint_A", lane=-6.476),
    "pulley_A2": PulleySpec("pulley_A2", radius=5.5, width=3.0, shaft="joint_A", lane=+3.257),
    "pulley_A3": PulleySpec("pulley_A3", radius=8.0, width=3.0, shaft="joint_A", lane=+5.109),

    "pulley_11": PulleySpec("pulley_11", radius=8.0, width=6.0, shaft="joint_1", lane=-6.476),
    "pulley_12": PulleySpec("pulley_12", radius=5.5, width=3.0, shaft="joint_1", lane=-3.252),
    "pulley_13": PulleySpec("pulley_13", radius=5.0, width=3.0, shaft="joint_1", lane=+3.257),
    "pulley_14": PulleySpec("pulley_14", radius=8.0, width=6.0, shaft="joint_1", lane=+5.109),

    "pulley_B1": PulleySpec("pulley_B1", radius=5.0, width=3.0, shaft="joint_B", lane=-3.252),
    "pulley_B2": PulleySpec("pulley_B2", radius=5.0, width=3.0, shaft="joint_B", lane=0.0),

    "pulley_21": PulleySpec("pulley_21", radius=5.0, width=3.0, shaft="joint_2", lane=-3.252),
    "pulley_22": PulleySpec("pulley_22", radius=5.0, width=3.0, shaft="joint_2", lane=0.0),
    "pulley_23": PulleySpec("pulley_23", radius=8.0, width=6.0, shaft="joint_2", lane=+3.257),

    "pulley_C1": PulleySpec("pulley_C1", radius=5.0, width=3.0, shaft="joint_C", lane=0.0),
    "pulley_31": PulleySpec("pulley_31", radius=8.0, width=6.0, shaft="joint_3", lane=-3.257),
}

DRUM_AXIS = np.array([0,1,0], float)
SPLAY_DRUM_AXIS = np.array([0,0,1], float)

DRUMS: dict[str, MotorDrumSpec] = {
    "drum_SPLAY_A": MotorDrumSpec("drum_SPLAY_A", tendon="SPLAY_A", radius=6.0, direction=+1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, -12.0, 0.0], float),
                                 axis_local=SPLAY_DRUM_AXIS),
    "drum_SPLAY_B": MotorDrumSpec("drum_SPLAY_B", tendon="SPLAY_B", radius=6.0, direction=-1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, +12.0, 0.0], float),
                                 axis_local=SPLAY_DRUM_AXIS),
    "drum_MCP_EXT": MotorDrumSpec("drum_MCP_EXT", tendon="MCP_EXT", radius=6.0, direction=+1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, -8.0, -1.40], float),
                                 axis_local=DRUM_AXIS),
    "drum_PIP_EXT": MotorDrumSpec("drum_PIP_EXT", tendon="PIP_EXT", radius=6.0, direction=+1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, -5.0, -1.40], float),
                                 axis_local=DRUM_AXIS),
    "drum_DIP_FLX": MotorDrumSpec("drum_DIP_FLX", tendon="DIP_FLX", radius=6.0, direction=+1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, +5.0, -1.40], float),
                                 axis_local=DRUM_AXIS),
    "drum_MCP_FLX": MotorDrumSpec("drum_MCP_FLX", tendon="MCP_FLX", radius=6.0, direction=+1.0,
                                 anchor_frame="O0", center_local=np.array([-40.0, +8.0, -1.40], float),
                                 axis_local=DRUM_AXIS),
}

ENDPOINTS: dict[str, AnchoredPoint] = {
    "mcp_ext_mid": AnchoredPoint("mcp_ext_mid", frame="O0", p_local=np.array([+6.0, -6.0, 0.0], float)),
    "mcp_ext_end": AnchoredPoint("mcp_ext_end", frame="O1", p_local=np.array([0.25*LINK_LENGTHS["proximal"], 0.0, 0.0], float)),

    "pip_ext_mid": AnchoredPoint("pip_ext_mid", frame="O0", p_local=np.array([+3.0, +1.5, 0.0], float)),
    "pip_ext_end": AnchoredPoint("pip_ext_end", frame="O2", p_local=np.array([0.25*LINK_LENGTHS["middle"], 0.0, 0.0], float)),

    "dip_flx_mid": AnchoredPoint("dip_flx_mid", frame="O0", p_local=np.array([+3.0, -1.5, 0.0], float)),
    "dip_flx_end": AnchoredPoint("dip_flx_end", frame="O3", p_local=np.array([FINGERTIP_OFFSET, 0.0, 0.0], float)),

    "mcp_flx_mid": AnchoredPoint("mcp_flx_mid", frame="O0", p_local=np.array([+6.0, +6.0, 0.0], float)),
    "mcp_flx_end": AnchoredPoint("mcp_flx_end", frame="O1", p_local=np.array([0.25*LINK_LENGTHS["proximal"], 0.0, 0.0], float)),

    "splay_a_end": AnchoredPoint("splay_a_end", frame="O0", p_local=np.array([0.5*LINK_LENGTHS["splay"], -6.0, -14.5], float)),
    "splay_b_end": AnchoredPoint("splay_b_end", frame="O0", p_local=np.array([0.5*LINK_LENGTHS["splay"], +6.0, -11.5], float)),
}

TENDON_ORDER = ["SPLAY_A", "SPLAY_B", "MCP_EXT", "PIP_EXT", "DIP_FLX", "MCP_FLX"]

TENDONS: dict[str, TendonPathSpec] = {
    "SPLAY_A": TendonPathSpec("SPLAY_A", items=[
        "DRUM:drum_SPLAY_A",
        TendonContact("pulley_01", +1, kind="fixed", spool_sign=+1.0),
    ]),
    "SPLAY_B": TendonPathSpec("SPLAY_B", items=[
        "DRUM:drum_SPLAY_B",
        TendonContact("pulley_04", -1, kind="fixed", spool_sign=-1.0),
    ]),
    "MCP_EXT": TendonPathSpec("MCP_EXT", items=[
        "DRUM:drum_MCP_EXT",
        TendonContact("pulley_Z1", -1),
        TendonContact("pulley_01", +1),
        "EP:mcp_ext_mid",
        TendonContact("pulley_A1", -1),
        TendonContact("pulley_11", +1, kind="fixed"),
    ]),
    "PIP_EXT": TendonPathSpec("PIP_EXT", items=[
        "DRUM:drum_PIP_EXT",
        TendonContact("pulley_Z2", +1),
        TendonContact("pulley_02", -1),
        "EP:pip_ext_mid",
        TendonContact("pulley_A2", -1),
        TendonContact("pulley_13", +1),
        TendonContact("pulley_23", -1, kind="fixed"),
    ]),
    "DIP_FLX": TendonPathSpec("DIP_FLX", items=[
        "DRUM:drum_DIP_FLX",
        TendonContact("pulley_Z3", -1),
        TendonContact("pulley_03", +1),
        "EP:dip_flx_mid",
        TendonContact("pulley_12", +1),
        TendonContact("pulley_B1", +1),
        TendonContact("pulley_21", -1),
        TendonContact("pulley_31", -1, kind="fixed", spool_sign=-1.0),
    ]),
    "MCP_FLX": TendonPathSpec("MCP_FLX", items=[
        "DRUM:drum_MCP_FLX",
        TendonContact("pulley_Z4", +1),
        TendonContact("pulley_04", -1),
        "EP:mcp_flx_mid",
        TendonContact("pulley_A3", +1),
        TendonContact("pulley_14", -1, kind="fixed"),
    ]),
}