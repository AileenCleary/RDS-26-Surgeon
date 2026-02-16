from math import dist
from shaft_analysis import pulley, PulleyPose, TendonContact, TendonPath
from components import Shaft, BearingBase, Bearing

"""
Global origin is centered [x,y] with the splay shaft, 
with z set as the center of the MCP joint.

Explicitly:
    - +x distal
    - +y right
    - +z down

Where the splay joint axis is z and the others are y in the global frame.
"""
#-----------------------------

splay_link_length = 20
proximal_link_length = 20
middle_link_length = 20
distal_link_length = 20

LINK_LENGTHS = [splay_link_length, proximal_link_length, middle_link_length, distal_link_length]
FINGERTIP_OFFSET = 10
COUPLING_RATIO = 0.7

# automate this later
tendon_y_axes = [-4.5,-1.5,0.0,1.5,4.5]

#-----------------------------
"""IMPOSED CONSTRAINTS: Fill out/add as needed."""
MIN_BORE_MM = 1.0 # ***
MIN_DYNAMIC_C_N = 1.0 # ***
#-----------------------------
"""BEARING CANDIDATE(S)"""
BEARING_CANDIDATE = [
    {
        "part": "57155K339",
        "inner_diameter_mm": 1.016,
        "outer_diameter_mm": 3.175,
        "width_mm": 1.19,
        "C_N": 88.96, # *** Fill in from datasheet, needed for deciding (N).
        "C0_N": 22.24, # Optional static rating.

        # Equivalent load factors
        "X": 1.0,
        "Y": 0.0,
        "p": 3.0,
        "type": "deep_groove_ball",
        "notes": "Fill in C/C0 from catalogue.",
    }, # Add more candidates.
]
#-----------------------------
"""BASE COMPONENTS"""
bearing_base = BearingBase(
    inner_diameter=1.016,
    outer_diameter=3.175,
    shaft_diameter=1.016,
    width=1.19,
)

pulley_base = pulley(
    radius=10,
    width=3,
)
#-----------------------------
joint_0 = Shaft(
    diameter=10,
    length=25,
    center=[0.0,0.0,0.0],
    axis=[0.0,0.0,1.0],
    alias="splay"
)
joint_Z = Shaft(
    diameter=10,
    length=25,
    center=[-(splay_link_length),0.0,0.0],
    axis=[0.0,0.0,1.0],
    alias="splay idler"
)
joint_A = Shaft(
    diameter=10,
    length=25,
    center=[(splay_link_length/2),0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="MCP idler"
)
joint_1 = Shaft(
    diameter=10,
    length=25,
    center=[splay_link_length,0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="MCP"
)
joint_B = Shaft(
    diameter=10,
    length=25,
    center=[joint_1.center[0]+proximal_link_length/2,0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="PIP Idler"
)
joint_2 = Shaft(
    diameter=10,
    length=25,
    center=[joint_1.center[0]+proximal_link_length,0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="PIP"
)
joint_C = Shaft(
    diameter=10,
    length=25,
    center=[joint_2.center[0]+middle_link_length/2,0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="DIP Idler"
)
joint_3 = Shaft(
    diameter=10,
    length=25,
    center=[joint_2.center[0]+middle_link_length,0.0,0.0],
    axis=[0.0,1.0,0.0],
    alias="DIP"
)

bearing_A0 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_A
)
bearing_A1 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_A
)
bearing_B0 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_B
)

bearing_B1 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_B
)

bearing_10 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_1
)

bearing_11 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_1
)

bearing_20 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_2
)

bearing_21 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_2
)

bearing_C0 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_C
)

bearing_C1 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_C
)

bearing_30 = Bearing(
    side=-1,
    base=bearing_base,
    shaft=joint_3
)

bearing_31 = Bearing(
    side=1,
    base=bearing_base,
    shaft=joint_3
)

pulley_01 = PulleyPose(
    name="pulley_01",
    shaft=joint_0,
    base=pulley_base,
    tendon=0,
    tendon_y_axes=tendon_y_axes,
)
pulley_02 = PulleyPose(
    name="pulley_02",
    shaft=joint_0,
    base=pulley_base,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)
pulley_03 = PulleyPose(
    name="pulley_03",
    shaft=joint_0,
    base=pulley_base,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_04 = PulleyPose(
    name="pulley_04",
    shaft=joint_0,
    base=pulley_base,
    tendon=4,
    tendon_y_axes=tendon_y_axes,
)
pulley_Z1 = PulleyPose(
    name="pulley_Z1",
    shaft=joint_Z,
    base=pulley_base,
    tendon=0,
    tendon_y_axes=tendon_y_axes,
)
pulley_Z2 = PulleyPose(
    name="pulley_Z2",
    shaft=joint_Z,
    base=pulley_base,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)
pulley_Z3 = PulleyPose(
    name="pulley_Z3",
    shaft=joint_Z,
    base=pulley_base,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_Z4 = PulleyPose(
    name="pulley_Z4",
    shaft=joint_Z,
    base=pulley_base,
    tendon=4,
    tendon_y_axes=tendon_y_axes,
)
pulley_A1 = PulleyPose(
    name="pulley_A1",
    shaft=joint_A,
    base=pulley_base,
    tendon=0,
    tendon_y_axes=tendon_y_axes,
)

pulley_A2 = PulleyPose(
    name="pulley_A2",
    base=pulley_base,
    shaft=joint_A,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)

pulley_A3 = PulleyPose(
    name="pulley_A3",
    base=pulley_base,
    shaft=joint_A,
    tendon=4,
    tendon_y_axes=tendon_y_axes,
)
pulley_11 = PulleyPose(
    name="pulley_11",
    base=pulley_base,
    shaft=joint_1,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_12 = PulleyPose(
    name="pulley_12",
    base=pulley_base,
    shaft=joint_1,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_13 = PulleyPose(
    name="pulley_13",
    base=pulley_base,
    shaft=joint_1,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)
pulley_14 = PulleyPose(
    name="pulley_14",
    base=pulley_base,
    shaft=joint_1,
    tendon=4,
    tendon_y_axes=tendon_y_axes,
)
pulley_B1 = PulleyPose(
    name="pulley_B1",
    base=pulley_base,
    shaft=joint_B,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)
pulley_21 = PulleyPose(
    name="pulley_21",
    base=pulley_base,
    shaft=joint_2,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_22 = PulleyPose(
    name="pulley_22",
    base=pulley_base,
    shaft=joint_2,
    tendon=2,
    tendon_y_axes=tendon_y_axes,
)
pulley_23 = PulleyPose(
    name="pulley_23",
    base=pulley_base,
    shaft=joint_2,
    tendon=3,
    tendon_y_axes=tendon_y_axes,
)
pulley_C1 = PulleyPose(
    name="pulley_C1",
    base=pulley_base,
    shaft=joint_C,
    tendon=2,
    tendon_y_axes=tendon_y_axes,
)
pulley_31 = PulleyPose(
    name="pulley_31",
    base=pulley_base,
    shaft=joint_3,
    tendon=1,
    tendon_y_axes=tendon_y_axes,
)
pulley_32 = PulleyPose(
    name="pulley_32",
    base=pulley_base,
    shaft=joint_3,
    tendon=2,
    tendon_y_axes=tendon_y_axes,
)
#-----------------------------
"""HELPERS"""
ALL_PULLEYS = {
    p.name: p for p in [
        pulley_Z1, pulley_Z2, pulley_Z3, pulley_Z4,
        pulley_01, pulley_02, pulley_03, pulley_04,
        pulley_A1, pulley_A2, pulley_A3,
        pulley_11, pulley_12, pulley_13, pulley_14,
        pulley_B1,
        pulley_21, pulley_22, pulley_23,
        pulley_C1,
        pulley_31, pulley_32,
    ]
}

BEARINGS_BY_SHAFT = {
    "A": (bearing_A0, bearing_A1, joint_A),
    "MCP": (bearing_10, bearing_11, joint_1),
    "B": (bearing_B0, bearing_B1, joint_B),
    "PIP": (bearing_20, bearing_21, joint_2),
    "C": (bearing_C0, bearing_C1, joint_C),
    "DIP": (bearing_30, bearing_31, joint_3),
}

SHAFTS = {
    "joint_Z": joint_Z,
    "joint_0": joint_0,
    "joint_A": joint_A,
    "joint_1": joint_1,
    "joint_B": joint_B,
    "joint_2": joint_2,
    "joint_C": joint_C,
    "joint_3": joint_3,
}

# *** Clarify tendon signs and all tendons.
TENDONS = {
    "MCP_extensor_demo": TendonPath(
        name="MCP_extensor_demo",
        contacts=[
            TendonContact("pulley_Z1", -1),
            TendonContact("pulley_01", +1),
            TendonContact("pulley_A1", -1),
            TendonContact("pulley_11", +1),
        ],
    ),
    "PIP_flexor_demo": TendonPath(
        name="PIP_flexor_demo",
        contacts=[
            TendonContact("pulley_Z2", +1),
            TendonContact("pulley_02", -1),
            TendonContact("pulley_A2", -1),
            TendonContact("pulley_13", +1),
            TendonContact("pulley_B1", +1),
            TendonContact("pulley_23", -1),
        ]
    )
}

TENDON_ORDER = [
    "SPLAY_A",
    "MCP_extensor_demo",
    "DIP_extensor",
    "INTERNAL",
    "PIP_flexor_demo",
    "MCP_flexor",
    "SPLAY_B",
]

TENDON_INDEX = {name:i for i,name in enumerate(TENDON_ORDER)}
