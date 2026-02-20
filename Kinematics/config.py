from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np # type: ignore

from components import Shaft, ShaftBase, BearingBase, Bearing, PulleyBase, Pulley, MotorDrum
from tendon_types import TendonContact, TendonEndpoint, TendonPath

"""
Global origin is centered [x,y] with the splay shaft, 
with z set as the center of the MCP joint.

Explicitly:
    - +x distal
    - +y right
    - +z down

Where the splay joint axis is z and the others are y in the global frame.
"""

#-------------------------------------------------
# Geometry
#-------------------------------------------------
splay_link_length = 25.0
proximal_link_length = 40.0
middle_link_length = 25.0
distal_link_length = 25.0

LINK_LENGTHS = [splay_link_length, proximal_link_length, middle_link_length, distal_link_length]
FINGERTIP_OFFSET = 10.0
COUPLING_RATIO = 0.7
#-------------------------------------------------
# Bases
#-------------------------------------------------
BEARING_BASES: Dict[str, BearingBase] = {
    "bearing_part_name": BearingBase(
        inner_diameter=4.0,
        outer_diameter=11.0,
        shaft_diameter=4.0,
        width=4.0,
        X=1.0,
        Y=0.0,
        p=3.0,
        C=0.0,
        C0=0.0, # Fill in the load ratings.
    ),
}

PULLEY_BASES: Dict[str, PulleyBase] = {
    "pulley_part_name": PulleyBase(
        radius=6.0,
        width=3.0
    ),
}

SHAFT_BASES: Dict[str, ShaftBase] = {
    "shaft_base_horizontal": ShaftBase(
        diameter=4.0,
        length=25.0,
        axis=np.array([0.0, 1.0, 0.0])
    ),
    "shaft_base_vertical": ShaftBase(
        diameter=4.0,
        length=25.0,
        axis=np.array([0.0, 0.0, 1.0])
    ),
}
#-------------------------------------------------
# Shafts
#-------------------------------------------------
def make_shaft(
        base_key: str,
        name: str,
        center: np.ndarray,
        dof_row: int | None = None,
        alias: str | None = None,
) -> Shaft:
    if base_key not in SHAFT_BASES:
        raise KeyError("")
    center = np.asarray(center, dtype=float).reshape(3)
    s = Shaft(
        base=SHAFT_BASES[base_key],
        name=name,
        center=center,
        dof_row=dof_row,
        alias=alias,
    )
    return s

joint_Z = make_shaft(
    base_key="shaft_base_vertical",
    name="joint_Z",
    center=[-(splay_link_length),0.0,0.0],
    dof_row=None,
    alias="splay idler"
)
joint_0 = make_shaft(
    base_key="shaft_base_vertical",
    name="joint_0",
    center=[0.0,0.0,0.0],
    dof_row = 0,
    alias="splay"
)
joint_A = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_A",
    center=[(splay_link_length/2),0.0,0.0],
    dof_row = None,
    alias="MCP idler"
)
joint_1 = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_1",
    center=[splay_link_length,0.0,0.0],
    dof_row = 1,
    alias="MCP"
)
joint_B = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_B",
    center=[joint_1.center[0]+proximal_link_length/2,0.0,0.0],
    dof_row=None,
    alias="PIP Idler"
)
joint_2 = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_2",
    center=[joint_1.center[0]+proximal_link_length,0.0,0.0],
    dof_row = 2,
    alias="PIP"
)
joint_C = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_C",
    center=[joint_2.center[0]+middle_link_length/2,0.0,0.0],
    dof_row=None,
    alias="DIP Idler"
)
joint_3 = make_shaft(
    base_key="shaft_base_horizontal",
    name="joint_3",
    center=[joint_2.center[0]+middle_link_length,0.0,0.0],
    dof_row = 3,
    alias="DIP"
)

SHAFTS: Dict[str, Shaft] = {
    "joint_Z": joint_Z,
    "joint_0": joint_0,
    "joint_A": joint_A,
    "joint_1": joint_1,
    "joint_B": joint_B,
    "joint_2": joint_2,
    "joint_C": joint_C,
    "joint_3": joint_3,
}
#-------------------------------------------------
# Bearings
#-------------------------------------------------
def make_bearing(
        base_key: str,
        shaft: Shaft,
        side: int,
        alias: str | None = None,
) -> Bearing:
    if base_key not in BEARING_BASES:
        raise KeyError("")
    b = Bearing(
        base=BEARING_BASES[base_key],
        shaft=shaft,
        side=side,
        alias=alias,
    )
    return b

bearing_Z0 = make_bearing(
    base_key="bearing_part_name",
    side=-1,
    shaft=joint_Z,
)
bearing_Z1 = make_bearing(
    base_key="bearing_part_name",
    side=1,
    shaft=joint_Z,
)
bearing_00 = make_bearing(
    base_key="bearing_part_name",
    side=-1,
    shaft=joint_0
)
bearing_01 = make_bearing(
    base_key="bearing_part_name",
    side=1,
    shaft=joint_0,
)
bearing_A0 = make_bearing(
    base_key="bearing_part_name",
    side=-1,
    shaft=joint_A
)
bearing_A1 = make_bearing(
    base_key="bearing_part_name",
    side=1,
    shaft=joint_A
)
bearing_B0 = make_bearing(
    side=-1,
    base_key="bearing_part_name",
    shaft=joint_B
)
bearing_B1 = make_bearing(
    side=1,
    base_key="bearing_part_name",
    shaft=joint_B
)
bearing_10 = make_bearing(
    side=-1,
    base_key="bearing_part_name",
    shaft=joint_1
)
bearing_11 = make_bearing(
    side=1,
    base_key="bearing_part_name",
    shaft=joint_1
)
bearing_20 = make_bearing(
    side=-1,
    base_key="bearing_part_name",
    shaft=joint_2
)
bearing_21 = make_bearing(
    side=1,
    base_key="bearing_part_name",
    shaft=joint_2
)
bearing_C0 = make_bearing(
    side=-1,
    base_key="bearing_part_name",
    shaft=joint_C
)
bearing_C1 = make_bearing(
    side=1,
    base_key="bearing_part_name",
    shaft=joint_C
)
bearing_30 = make_bearing(
    side=-1,
    base_key="bearing_part_name",
    shaft=joint_3
)
bearing_31 = make_bearing(
    side=1,
    base_key="bearing_part_name",
    shaft=joint_3
)

BEARINGS_BY_SHAFT = {
    "Z": (bearing_Z0, bearing_Z1, joint_Z),
    "0": (bearing_00, bearing_01, joint_0),
    "A": (bearing_A0, bearing_A1, joint_A),
    "MCP": (bearing_10, bearing_11, joint_1),
    "B": (bearing_B0, bearing_B1, joint_B),
    "PIP": (bearing_20, bearing_21, joint_2),
    "C": (bearing_C0, bearing_C1, joint_C),
    "DIP": (bearing_30, bearing_31, joint_3),
}
#-------------------------------------------------
# Pulleys
#-------------------------------------------------
PULLEY_ROLE: Dict[str, str] = {}

def make_pulley(
        base_key: str,
        name: str,
        lane: float,
        shaft: Shaft,
        role: str,
        mu: float = 0.0,
        shaft_name: Optional[str] = "",
        tangent_points: Optional[List[np.ndarray]] = None,
) -> Pulley:
    if base_key not in PULLEY_BASES:
        raise KeyError("")
    p = Pulley(
            base=PULLEY_BASES[base_key],
            name=name,
            lane=lane,
            shaft=shaft,
            role=role,
            mu=mu,
            shaft_name=shaft_name,
            tangent_points=tangent_points,
    )
    PULLEY_ROLE[name] = role
    return p

pulley_ZA = make_pulley(
    base_key="pulley_part_name",
    name="pulley_ZA",
    lane=1.5,
    shaft=joint_Z,
    role="idler"
    #tendon=A,
)
pulley_ZB = make_pulley(
    base_key="pulley_part_name",
    name="pulley_ZB",
    lane=-1.5,
    shaft=joint_Z,
    role="idler"
    #tendon=A,
)
pulley_Z1 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_Z1",
    lane=-7.5,
    shaft=joint_Z,
    role="idler"
    #tendon=0,
)
pulley_Z2 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_Z2",
    lane=-4.5,
    shaft=joint_Z,
    role="idler",
    # tendon=3,
)
pulley_Z3 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_Z3",
    lane=4.5,
    shaft=joint_Z,
    role="idler",
    # tendon=1,
)
pulley_Z4 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_Z4",
    lane=7.5,
    shaft=joint_Z,
    role="idler"
    # tendon=4,
)
pulley_0A = make_pulley(
    base_key="pulley_part_name",
    name="pulley_0A",
    lane=1.5,
    shaft=joint_0,
    role="idler"
    # tendon=0,
)
pulley_0B = make_pulley(
    base_key="pulley_part_name",
    name="pulley_0B",
    lane=-1.5,
    shaft=joint_0,
    role="idler"
    # tendon=0,
)
pulley_01 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_01",
    lane=-7.5,
    shaft=joint_0,
    role="idler"
    # tendon=0,
)
pulley_02 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_02",
    lane=-4.5,
    shaft=joint_0,
    role="idler"
    # tendon=3,
)
pulley_03 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_03",
    lane=4.5,
    shaft=joint_0,
    role="idler"
    # tendon=1,
)
pulley_04 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_04",
    lane=7.5,
    shaft=joint_0,
    role="idler"
    # tendon=4,
)
pulley_A1 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_A1",
    lane=-6.0,
    shaft=joint_A,
    role="idler"
    # tendon=0,
)
pulley_A2 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_A2",
    lane=3.0,
    shaft=joint_A,
    role="idler"
    # tendon=3,
)
pulley_A3 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_A3",
    lane=6.0,
    shaft=joint_A,
    role="idler"
    # tendon=4,
)
pulley_11 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_11",
    lane=-3.0,
    shaft=joint_1,
    role="idler"
    # tendon=1,
)
pulley_12 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_12",
    lane=-3.0,
    shaft=joint_1,
    role="idler"
    # tendon=1,
)
pulley_13 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_13",
    lane=3.0,
    shaft=joint_1,
    role="idler"
    # tendon=3,
)
pulley_14 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_14",
    lane=6.0,
    shaft=joint_1,
    role="idler"
    # tendon=4,
)
pulley_B1 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_B1",
    lane=3.0,
    shaft=joint_B,
    role="idler"
    # tendon=3,
)
pulley_21 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_21",
    lane=-3.0,
    shaft=joint_2,
    role="idler"
    # tendon=1,
)
pulley_22 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_22",
    lane=0.0,
    shaft=joint_2,
    role="idler"
    # tendon=2,
)
pulley_23 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_23",
    lane=3.0,
    shaft=joint_2,
    role="idler"
    # tendon=3,
)
pulley_C1 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_C1",
    lane=0.0,
    shaft=joint_C,
    role="idler"
    # tendon=2,
)
pulley_31 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_31",
    lane=-3.0,
    shaft=joint_3,
    role="idler"
    # tendon=1,
)
pulley_32 = make_pulley(
    base_key="pulley_part_name",
    name="pulley_32",
    lane=0.0,
    shaft=joint_3,
    role="idler"
    # tendon=2,
)

ALL_PULLEYS = {
    p.name: p for p in [
        pulley_Z1, pulley_Z2, pulley_ZA, pulley_ZB, pulley_Z3, pulley_Z4,
        pulley_01, pulley_02, pulley_0A, pulley_0B, pulley_03, pulley_04,
        pulley_A1, pulley_A2, pulley_A3,
        pulley_11, pulley_12, pulley_13, pulley_14,
        pulley_B1,
        pulley_21, pulley_22, pulley_23,
        pulley_C1,
        pulley_31, pulley_32,
    ]
}

DRUM_AXIS = np.array([0.0, 1.0, 0.0], dtype=float)

drum_SPLAY_A = MotorDrum(
    name="drum_SPLAY_A",
    radius=6.0,
    center=pulley_ZA.center + np.array([-40.0, -pulley_ZA.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="SPLAY_A",
    direction=+1.0,
)
drum_SPLAY_B = MotorDrum(
    name="drum_SPLAY_B",
    radius=6.0,
    center=pulley_ZB.center + np.array([-40.0, pulley_ZB.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="SPLAY_B",
    direction=-1.0,
)
drum_MCP_EXT = MotorDrum(
    name="drum_MCP_EXT",
    radius=6.0,
    center=pulley_Z1.center + np.array([-40.0, -pulley_Z1.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="MCP_EXT",
    direction=1.0,
)
drum_PIP_FLX = MotorDrum(
    name="drum_PIP_FLX",
    radius=6.0,
    center=pulley_Z2.center + np.array([-40.0, -pulley_Z2.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="PIP_FLX",
    direction=1.0,
)
drum_DIP_EXT = MotorDrum(
    name="drum_DIP_EXT",
    radius=6.0,
    center=pulley_Z3.center + np.array([-40.0, pulley_Z3.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="DIP_EXT",
    direction=1.0,
)
drum_MCP_FLX = MotorDrum(
    name="drum_MCP_FLX",
    radius=6.0,
    center=pulley_Z4.center + np.array([-40.0, pulley_Z4.radius, 0.0]),
    axis=DRUM_AXIS,
    tendon_name="MCP_FLX",
    direction=1.0,
)

MOTORS: Dict[str, List[MotorDrum]] = {
    "SPLAY_motor": [drum_SPLAY_A, drum_SPLAY_B],
    "MCP_EXT_motor": [drum_MCP_EXT],
    "MCP_FLX_motor": [drum_MCP_FLX],
    "PIP_FLX_motor": [drum_PIP_FLX],
    "DIP_EXT_motor": [drum_DIP_EXT],
}

DRUMS_BY_NAME: Dict[str, MotorDrum] = {
    d.name: d for ds in MOTORS.values() for d in ds
}
#-------------------------------------------------
# Tendons and Endpoints
#-------------------------------------------------
TENDON_ORDER = [
    "SPLAY_A",
    "MCP_EXT",
    "DIP_EXT",
    "INTERNAL",
    "PIP_FLX",
    "MCP_FLX",
    "SPLAY_B",
]

TENDON_INDEX = {name:i for i,name in enumerate(TENDON_ORDER)}

def ep_from_pulley_offset(
        *,
        pulley: Pulley,
        offset: np.ndarray,
        kind: str,
        tendon_name: str,
        tendon_index: int,
) -> TendonEndpoint:
    return TendonEndpoint(
        coordinates=np.array(pulley.center, dtype=float) + np.array(offset, dtype=float),
        kind=kind,
        tendon=tendon_index,
        tendon_name=tendon_name,
    )

def ep_from_drum(
        *,
        drum: MotorDrum,
        kind: str,
) -> TendonEndpoint:
    t_idx = TENDON_INDEX.get(drum.tendon_name, -1)
    return TendonEndpoint(
        coordinates=np.array(drum.center, dtype=float),
        kind=kind,
        tendon=t_idx,
        tendon_name=drum.tendon_name,
    )

mcp_extensor_mid = ep_from_pulley_offset(
    pulley=pulley_01,
    offset=([pulley_01.radius,-pulley_01.radius,0.0]),
    kind="mid",
    tendon_index=TENDON_INDEX["MCP_EXT"],
    tendon_name="MCP_EXT",
)
mcp_extensor_end = ep_from_pulley_offset(
    pulley=pulley_11,
    offset=np.array([0.25*proximal_link_length,0.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["MCP_EXT"],
    tendon_name="MCP_EXT",
)

pip_flexor_mid = ep_from_pulley_offset(
    pulley=pulley_02,
    offset=np.array([pulley_02.radius,0.5*pulley_02.radius,0.0]),
    kind="mid",
    tendon_index=TENDON_INDEX["PIP_FLX"],
    tendon_name="PIP_FLX",
)
pip_flexor_end = ep_from_pulley_offset(
    pulley=pulley_23,
    offset=np.array([0.25*middle_link_length,0.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["PIP_FLX"],
    tendon_name="PIP_FLX",
)

INTERNAL_start = ep_from_pulley_offset(
    pulley=pulley_22,
    offset=np.array([-proximal_link_length*0.5,0.0,0.0]),
    kind="start",
    tendon_index=TENDON_INDEX["INTERNAL"],
    tendon_name="INTERNAL",
)
INTERNAL_end = ep_from_pulley_offset(
    pulley=pulley_32,
    offset=np.array([0.25*distal_link_length,0.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["INTERNAL"],
    tendon_name="INTERNAL",
)

dip_extensor_mid = ep_from_pulley_offset(
    pulley=pulley_03,
    offset=np.array([pulley_03.radius,-0.5*pulley_03.radius,0.0]),
    kind="mid",
    tendon_index=TENDON_INDEX["DIP_EXT"],
    tendon_name="DIP_EXT",
)
dip_extensor_end = ep_from_pulley_offset(
    pulley=pulley_31,
    offset=np.array([0.25*distal_link_length,0.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["DIP_EXT"],
    tendon_name="DIP_EXT",
)

mcp_flexor_mid = ep_from_pulley_offset(
    pulley=pulley_04,
    offset=np.array([pulley_04.radius,pulley_04.radius,0.0]),
    kind="mid",
    tendon_index=TENDON_INDEX["MCP_FLX"],
    tendon_name="MCP_FLX",
)
mcp_flexor_end = ep_from_pulley_offset(
    pulley=pulley_14,
    offset=np.array([0.25*proximal_link_length,0.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["MCP_FLX"],
    tendon_name="MCP_FLX",
)

splay_a_end = ep_from_pulley_offset(
    pulley=pulley_0A,
    offset=np.array([0.5*splay_link_length,-6.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["SPLAY_A"],
    tendon_name="SPLAY_A",
)
splay_b_end = ep_from_pulley_offset(
    pulley=pulley_0B,
    offset=np.array([0.5*splay_link_length,6.0,0.0]),
    kind="end",
    tendon_index=TENDON_INDEX["SPLAY_B"],
    tendon_name="SPLAY_B",
)

splay_a_start = ep_from_drum(
    drum=drum_SPLAY_A,
    kind="start"
)
splay_b_start = ep_from_drum(
    drum=drum_SPLAY_B,
    kind="start"
)
mcp_ext_start = ep_from_drum(
    drum=drum_MCP_EXT,
    kind="start"
)
mcp_flx_start = ep_from_drum(
    drum=drum_MCP_FLX,
    kind="start"
)
dip_ext_start = ep_from_drum(
    drum=drum_DIP_EXT,
    kind="start"
)
pip_flx_start = ep_from_drum(
    drum=drum_PIP_FLX,
    kind="start"
)

TENDONS = {
    "SPLAY_A": TendonPath(
        name="SPLAY_A",
        contacts=[
            splay_a_start,
            TendonContact("pulley_ZA", +1),
            TendonContact("pulley_0A", +1),
            splay_a_end,
        ]
    ),
    "SPLAY_B": TendonPath(
        name="SPLAY_B",
        contacts=[
            splay_b_start,
            TendonContact("pulley_ZB", -1),
            TendonContact("pulley_0B", -1),
            splay_b_end,
        ]
    ),
    "MCP_EXT": TendonPath(
        name="MCP_EXT",
        contacts=[
            mcp_ext_start,
            TendonContact("pulley_Z1", -1),
            TendonContact("pulley_01", +1),
            mcp_extensor_mid,
            TendonContact("pulley_A1", -1),
            TendonContact("pulley_11", +1),
            mcp_extensor_end,
        ],
    ),
    "DIP_EXT": TendonPath(
        name="DIP_EXT",
        contacts=[
            dip_ext_start,
            TendonContact("pulley_Z3", -1),
            TendonContact("pulley_03", +1),
            dip_extensor_mid,
            TendonContact("pulley_12", +1),
            TendonContact("pulley_21", +1),
            TendonContact("pulley_31", +1),
            dip_extensor_end,
        ]
    ),
    "INTERNAL": TendonPath(
        name="INTERNAL",
        contacts=[
            INTERNAL_start,
            TendonContact("pulley_22", +1),
            TendonContact("pulley_C1", +1),
            TendonContact("pulley_32", -1),
            INTERNAL_end,
        ]
    ),
    "PIP_FLX": TendonPath(
        name="PIP_FLX",
        contacts=[
            pip_flx_start,
            TendonContact("pulley_Z2", +1),
            TendonContact("pulley_02", -1),
            pip_flexor_mid,
            TendonContact("pulley_A2", -1),
            TendonContact("pulley_13", +1),
            TendonContact("pulley_B1", +1),
            TendonContact("pulley_23", -1),
            pip_flexor_end,
        ]
    ),
    "MCP_FLX": TendonPath(
        name="MCP_FLX",
        contacts=[
            mcp_flx_start,
            TendonContact("pulley_Z4", +1),
            TendonContact("pulley_04", -1),
            mcp_flexor_mid,
            TendonContact("pulley_A3", +1),
            TendonContact("pulley_14", -1),
            mcp_flexor_end,
        ]
    ),
}

D_main = np.array([
    [+1.0, 1..0, 1.0, 1.0, 1.0, 1.0, +1.0],   # splay removed
    [0.0, +1.0, +1.0, 0.0, +1.0, +1.0, 0.0],   # MCP
    [0.0, 0.0, +1.0, +1.0, +1.0, 0.0, 0.0],    # PIPgen
])

