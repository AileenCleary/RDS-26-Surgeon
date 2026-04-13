import numpy as np
from rds_finger.types.pulleys import Pulley
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.bearings import Bearing

# ===================================================================
# Global Variables
# ===================================================================
GLOBAL_UP = np.array([0.0,0.0,1.0]) # global +z
CW = -1
CCW = 1

# ===================================================================
# Pulleys and Termination Points by Tendon
# ===================================================================

# BLUE
START1 = Point3D(np.array([-25.0, 32.0, 36.0]), "START", "PIP_EXT")
PULLEY1 = Pulley(np.array([7.0, 32.0, 36.0]), 10.0, [0.0, 0.0, 1.0], CCW, "PIP_EXT", "A")
PULLEY2 = Pulley(np.array([26.0, 32.0, 36.0]), 4.15, [0.0, 0.0, 1.0], CW, "PIP_EXT", "0")
MIDDLE1 = Point3D(np.array([33.0, 36.0, 36.0]), "MIDDLE", "PIP_EXT")
PULLEY3 = Pulley(np.array([50.0, 36.0, 22.5]), 13.6, [0.0, 1.0, 0.0], CCW, "PIP_EXT", "1")
PULLEY4 = Pulley(np.array([94.0, 36.0, 22.5]), 10.0, [0.0, 1.0, 0.0], CCW, "PIP_EXT", "2")
END1 = Point3D(np.array([104.0, 36.0, 28]), "END", "PIP_EXT")

# GREEN
START2 = Point3D(np.array([-25.0, 32.0, 30.150]), "START", "PIP_FLX")
PULLEY5 = Pulley(np.array([7.0, 32.0, 30.150]), 10.0, [0.0, 0.0, 1.0], CW, "PIP_FLX", "A")
PULLEY6 = Pulley(np.array([26.0, 32.0, 30.150]), 4.15, [0.0, 0.0, 1.0], CCW, "PIP_FLX", "0")
MIDDLE2 = Point3D(np.array([33.0, 29.35, 30.150]), "MIDDLE", "PIP_FLX")
PULLEY7 = Pulley(np.array([50.0, 29.350, 22.5]), 7.6, [0.0, 1.0, 0.0], CCW, "PIP_FLX", "1")
PULLEY8 = Pulley(np.array([94.0, 29.35, 22.5]), 10.0, [0.0, 1.0, 0.0], CW, "PIP_FLX", "2")
END2 = Point3D(np.array([104.0, 29.35, 28]), "END", "PIP_FLX")

# YELLOW
START3 = Point3D(np.array([-25.0, 32.0, 10.5]), "START", "MCP_FLX")
PULLEY9 = Pulley(np.array([7.0, 32.0, 10.5]), 10.0, [0.0, 0.0, 1.0], CCW, "MCP_FLX", "A")
PULLEY10 = Pulley(np.array([26.0, 32.0, 10.5]), 7.75, [0.0, 0.0, 1.0], CW, "MCP_FLX", "0")
MIDDLE3 = Point3D(np.array([35.0, 32, 22.5]), "MIDDLE", "MCP_FLX")
PULLEY11 = Pulley(np.array([50.0, 40, 22.5]), 13.5, [0.0, 1.0, 0.0], CW, "MCP_FLX", "1")
END3 = Point3D(np.array([64, 40, 30]), "END", "MCP_FLX")

# PURPLE
START4 = Point3D(np.array([-25.0, 32.0, 31.350]), "START", "MCP_EXT")
PULLEY12 = Pulley(np.array([7.0, 32.0, 31.350]), 10.0, [0.0, 0.0, 1.0], CW, "MCP_EXT", "A")
PULLEY13 = Pulley(np.array([26.0, 32.0, 31.350]), 4.15, [0.0, 0.0, 1.0], CCW, "MCP_EXT", "0")
MIDDLE4 = Point3D(np.array([33.0, 27, 31.350]), "MIDDLE", "MCP_EXT")
PULLEY14 = Pulley(np.array([50.0, 27, 22.5]), 10, [0.0, 1.0, 0.0], CCW, "MCP_EXT", "1")
END4 = Point3D(np.array([64, 27, 33]), "END", "MCP_EXT")

# BLACK: SPLAY A
START5 = Point3D(np.array([-25.0, 32.0, 14]), "START", "SPLAY_A")
PULLEY15 = Pulley(np.array([26, 32.0, 14]), 11.5, [0.0, 0.0, 1.0], CCW, "SPLAY_A", "0")
END5 = Point3D(np.array([12, 32.0, 14]), "END", "SPLAY_A")

# BLACK: SPLAY B
START6 = Point3D(np.array([-25.0, 32.0, 19]), "START", "SPLAY_B")
PULLEY16 = Pulley(np.array([26, 32.0, 19]), 11.5, [0.0, 0.0, 1.0], CW, "SPLAY_B", "0")
END6 = Point3D(np.array([12, 32.0, 19]), "END", "SPLAY_B")

# ORANGE
START7 = Point3D(np.array([72, 40, 28.5]), "START", "INTERNAL_FLX")
PULLEY17 = Pulley(np.array([94, 40, 22.5]), 7.7, [0.0, 0.0, 1.0], CCW, "MCP_INTERNAL_FLXEXT", "2")
PULLEY18 = Pulley(np.array([133, 40.0, 22.5]), 10, [0.0, 0.0, 1.0], CW, "INTERNAL_FLX", "3")
END7 = Point3D(np.array([150, 40, 33]), "END", "INTERNAL_FLX")

# RED
START8 = Point3D(np.array([86, 27, 28.5]), "START", "INTERNAL_EXT")
PULLEY19 = Pulley(np.array([94, 27, 22.5]), 7.1, [0.0, 0.0, 1.0], CW, "INTERNAL_EXT", "2")
PULLEY20 = Pulley(np.array([133, 27, 22.5]), 10, [0.0, 0.0, 1.0], CCW, "INTERNAL_EXT", "3")
END8 = Point3D(np.array([150, 27, 33]), "END", "INTERNAL_EXT")

# ===================================================================
# Bearings (Left & Right)
# ===================================================================
BEARING_A_L = Bearing(np.array([7.0, 32.0, 41]), [0.0, 0.0, 1.0], "A", "left", 266, 711)
BEARING_A_R = Bearing(np.array([7.0, 32.0, 1]), [0.0, 0.0, 1.0], "A", "right", 266, 711)
BEARING_0_L = Bearing(np.array([26.0, 32.0, 41]), [0.0, 0.0, 1.0], "0","left", 266, 711)
BEARING_0_R = Bearing(np.array([26.0, 32.0, 1]), [0.0, 0.0, 1.0], "0","right", 266, 711)
BEARING_1_L = Bearing(np.array([50, 12, 22.5]), [0.0, 1.0, 0.0], "1","left", 266, 711)
BEARING_1_R = Bearing(np.array([50, 47, 22.5]), [0.0, 1.0, 0.0], "1","right", 266, 711)
BEARING_2_L = Bearing(np.array([94, 12, 22.5]), [0.0, 1.0, 0.0], "2","left", 266, 711)
BEARING_2_R = Bearing(np.array([94, 47, 22.5]), [0.0, 1.0, 0.0], "2","right", 266, 711)
BEARING_3_L = Bearing(np.array([133, 12, 22.5]), [0.0, 1.0, 0.0], "3","left", 266, 711)
BEARING_3_R = Bearing(np.array([133, 47, 22.5]), [0.0, 1.0, 0.0], "3","right", 266, 711)

# ===================================================================
# Dictionaries
# ===================================================================
SHAFT_BEARINGS = {
    "A": {"left": BEARING_A_L, "right": BEARING_A_R},
    "0": {"left": BEARING_0_L, "right": BEARING_0_R},
    "1": {"left": BEARING_1_L, "right": BEARING_1_R},
    "2": {"left": BEARING_2_L, "right": BEARING_2_R},
    "3": {"left": BEARING_3_L, "right": BEARING_3_R},
}

TENDON_PATH = {
    "PIP_EXT": [START1, PULLEY1, PULLEY2, MIDDLE1, PULLEY3, PULLEY4, END1],
    "PIP_FLX": [START2, PULLEY5, PULLEY6, MIDDLE2, PULLEY7, PULLEY8, END2],
    "MCP_FLX": [START3, PULLEY9, PULLEY10, MIDDLE3, PULLEY11, END3],
    "MCP_EXT": [START4, PULLEY12, PULLEY13, MIDDLE4, PULLEY14, END4],
    # "SPLAY_A": [START5, PULLEY15, END5],
    # "SPLAY_B": [START6, PULLEY16, END6],
    "INTERNAL_FLX": [START7, PULLEY17, PULLEY18, END7],
    "INTERNAL_EXT": [START8, PULLEY19, PULLEY20, END8],
}

SHAFT_LOADS = {
    "A": [],
    "0": [],
    "1": [],
    "2": [],
    "3": [],
}

tendon_tensions = {
        "PIP_EXT": 1.0,
        "PIP_FLX": 113.00,
        "MCP_FLX": 159.9524,
        "MCP_EXT": 1.0,
        # "SPLAY_A": 0.0,
        # "SPLAY_B": 192.0,
        "INTERNAL_FLX": 1,
        "INTERNAL_EXT": 1,
    }