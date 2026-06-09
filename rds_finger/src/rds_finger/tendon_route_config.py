"""
tendon_route_config.py: Most recent tendon route configuration. Last updated: Aileen Cleary, 05/11/2026
"""

import numpy as np
from rds_finger.tensionability.route_configs import RouteConfig

"""
Tendon Mapping:
  1. PIP EXT (Blue)
  2. MCP EXT (Purple)
  3. PIP FLX (Green)
  4. MCP FLX (Yellow)
  5. Internal EXT (Red)
  6. Internal FLX (Orange)
  7. SPLAY A (Black)
"""

r29 = 2.9 # small blue
r64 = 6.4 # purple-pink mid-sized
r9 = 9 # large mcp splay 
r91 = 9.1 # mid-sized gray
r63 = 6.35 # pale yellow
r126 = 12.6 # large blue
r124 = 12.4 # large gray
r66 = 6.6 # gray mid

"""
Tendon Array Order (Left to Right):
  1. MCP FLX
  2. MCP EXT
  3. PIP FLX
  4. PIP EXT
  5. INTERNAL FLX
  6. INTERNAL EXT
  7. SPLAY A
  8. SPLAY B
"""

config = (
    np.array([
        [ -1.0,  1.0,  1.0, -1.0,  0.0,  0.0,  1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0,  0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0,  1.0, -1.0,  0.0,  0.0],
        [  0.0,  0.0,  0.0,  0.0, -1.0,  1.0,  0.0,  0.0],
    ]),
   np.array([
        [    r64,    r64,   r29,    r29,  0.0,  0.0,   r9,  r9],
        [   r124,    r91,   r63,   r126,  0.0,  0.0,  0.0, 0.0],
        [    0.0,    0.0,   r91,    r91,  r63,  r63,  0.0, 0.0],
        [    0.0,    0.0,   0.0,    0.0,   r9,   r9,  0.0, 0.0],
    ])
)

CONFIG = RouteConfig(
    D4=config[0],
    S4=config[1],
    desc=f"NEW Tendon Routing Configuration: 05/11/2026",
    name="CONFIG",
    tendon_names=[
        "MCP_flx",
        "MCP_ext",
        "PIP_flx",
        "PIP_ext",
        "INTERNAL_flx",
        "INTERNAL_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

config_no_int = (
    np.array([
        [ -1.0,  1.0,  1.0, -1.0,  1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0,  0.0,  0.0],
        [  0.0,  0.0,  0.0,  0.0,  0.0,  0.0],
    ]),
   np.array([
        [    r64,    r64,   r29,    r29,   r9,  r9],
        [   r124,    r91,   r63,   r126,  0.0, 0.0],
        [    0.0,    0.0,   r91,    r91,  0.0, 0.0],
        [    0.0,    0.0,   0.0,    0.0,  0.0, 0.0],
    ])
)

CONFIG_NO_INT = RouteConfig(
    D4=config_no_int[0],
    S4=config_no_int[1],
    desc=f"NEW Tendon Routing Configuration (without internal tendons): 05/11/2026",
    name="CONFIG",
    tendon_names=[
        "MCP_flx",
        "MCP_ext",
        "PIP_flx",
        "PIP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)