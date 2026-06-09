"""
tendon_config_legacy.py: Different tendon route configurations for comparison -- LEGACY. Last updated: Aileen Cleary, 05/11/2026
"""

import numpy as np
from rds_finger.tensionability.route_configs import RouteConfig

RG = 8.0 # general radius

config_v1_no_internal = (
    # mcp flx, dip flx, pip ext, mcp ext, splay A, splay B
    np.array([
        [ -1.0,  1.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  0.0,  0.0,  0.0,  0.0],
    ]),
    np.array([
        [  6.6,  2.7,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  0.0,  0.0,  0.0,  0.0],
    ])
)

config_v1_internal = (
    # mcp flx, dip flx, internal, pip ext, mcp ext, splay A, splay B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0,  0.0,  0.0],
    ]),
    np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  0.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  7.15,  0.0,  0.0,  0.0,  0.0],
    ])
)

config_v2_no_internal = (
    # mcp flx, dip flx, dip ext, mcp ext, splay A, splay B
    np.array([
        [ -1.0,  1.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0,  0.0],
    ]),
    np.array([
        [  6.6,  2.7,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  8.0,  0.0,  0.0,  0.0],
    ])
)

config_v2_internal = (
    # mcp flx, dip flx, internal, dip ext, mcp ext, splay A, splay B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  1.0,  0.0,  0.0,  0.0],
    ]),
   np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  0.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  7.15,  8.0,  0.0,  0.0,  0.0],
    ])
)

config_v3_internal = (
    # mcp flx, dip flx, internal, dip ext, splay A, splay B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  1.0,  0.0,  0.0],
    ]),
   np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6],
        [  8.0,  5.5,  0.0,  5.0,  0.0,  0.0],
        [  0.0,  5.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  8.0,  7.15,  8.0,  0.0,  0.0],
    ])
)

config_v3_internal = (
    # mcp flx, dip flx, internal, dip ext, splay A, splay B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  1.0,  0.0,  0.0],
    ]),
   np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6],
        [  RG,  RG,  0.0,  RG,  0.0,  0.0],
        [  0.0,  RG,  RG,  RG,  0.0,  0.0],
        [  0.0,  RG,  RG,  RG,  0.0,  0.0],
    ])
)

config_v3_no_internal = (
    # mcp flx, dip flx, dip ext, splay A, splay B
    np.array([
        [ -1.0,  1.0, -1.0,  1.0, -1.0],
        [ -1.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0,  1.0,  0.0,  0.0],
    ]),
   np.array([
        [  6.6,  2.7,  2.7,  6.6,  6.6],
        [  8.0,  5.5,  5.0,  0.0,  0.0],
        [  0.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  8.0,  8.0,  0.0,  0.0],
    ])
)

config_v4 = (
    # for first row (DOF SPLAY), I'll assume tendons have same routing as prev.
    # r1=r2=r3=8.0
    # green:dip flx, purple:mcp ext, blue:pip ext, red:internal, splay A, splay B
    np.array([
        [  1.0,  1.0, -1.0,  0.0, 1.0, -1.0],
        [ -1.0,  1.0, -1.0,  0.0, 0.0,  0.0],
        [ -1.0,  0.0,  1.0, -1.0, 0.0,  0.0],
        [ -1.0,  0.0,  0.0,  1.0, 0.0,  0.0],
    ]),
   np.array([
        [  2.7,  6.6,  2.7,  0.0, 6.6,  6.6],
        [   RG,   RG,   RG,  0.0, 0.0,  0.0],
        [   RG,  0.0,   RG,   RG, 0.0,  0.0],
        [   RG,  0.0,  0.0,   RG, 0.0,  0.0],
    ])
)

config_v5 = (
    # green:dip ext, purple:mcp flx, blue:pip flx, red:internal, splay A, splay B
    np.array([
        [ -1.0, -1.0,  1.0,  0.0, 1.0, -1.0],
        [  1.0, -1.0,  1.0,  0.0, 0.0,  0.0],
        [  1.0,  0.0, -1.0,  1.0, 0.0,  0.0],
        [  1.0,  0.0,  0.0, -1.0, 0.0,  0.0],
    ]),
   np.array([
        [  2.7,  6.6,  2.7,  0.0, 6.6,  6.6],
        [   RG,   RG,   RG,  0.0, 0.0,  0.0],
        [   RG,  0.0,   RG,   RG, 0.0,  0.0],
        [   RG,  0.0,  0.0,   RG, 0.0,  0.0],
    ])
)

config_v6 = (
    # 1 yellow:mcp flx, 2 purple:mcp ext, 3 green:pip flx, 4 blue:pip ext, orange:internal flx, red:internal ext, splay A, splay B
    np.array([
        [  1.0, -1.0,  1.0, -1.0,  0.0,  0.0, 1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0, 0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0,  1.0, -1.0, 0.0,  0.0],
        [  0.0,  0.0,  0.0,  0.0, -1.0,  1.0, 0.0,  0.0],
    ]),
   np.array([
        [  7.15,   7.15,   2.9,   2.9,  0.0,  0.0, 12.0, 12.0],
        [   9.0,    9.0,   6.3,  15.5,  0.0,  0.0,  0.0,  0.0],
        [   0.0,    0.0,   9.0,   9.0,  6.3,  6.3,  0.0,  0.0],
        [   0.0,    0.0,   0.0,   0.0,  9.0,  9.0,  0.0,  0.0],
    ])
)

config_v7 = (
    # alt version of julia's third option but tendon pair extends to dip instead of pip
    # for those signs I used the same as prev configs.
    # purple:mcp flx, magenta:mcp ext, blue:dip flx, cyan:dip ext, red:internal flx, orange:internal ext, splay A, splay B
    np.array([
        [ -1.0,  1.0,  1.0, -1.0,  0.0,  0.0, 1.0, -1.0],
        [ -1.0,  1.0,  1.0,  1.0,  0.0,  0.0, 0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0,  1.0, -1.0, 0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0, -1.0,  1.0, 0.0,  0.0],
    ]),
   np.array([
        [  6.7,  6.6,  2.7,  2.7,  0.0,  0.0, 6.6,  6.6],
        [   RG,   RG,   RG,   RG,  0.0,  0.0, 0.0,  0.0],
        [  0.0,  0.0,   RG,   RG,   RG,   RG, 0.0,  0.0],
        [  0.0,  0.0,   RG,   RG,   RG,   RG, 0.0,  0.0],
    ])
)

config_v8 = (
    # alt version of julia's third option but tendon pair extends to dip instead of pip
    # for those signs I used the same as prev configs.
    # purple:mcp flx, magenta:mcp ext, blue:dip ext, cyan:dip flx, red:internal flx, orange:internal ext, splay A, splay B
    np.array([
        [ -1.0,  1.0, -1.0,  1.0,  0.0,  0.0, 1.0, -1.0],
        [ -1.0,  1.0,  1.0, -1.0,  0.0,  0.0, 0.0,  0.0],
        [  0.0,  0.0, -1.0,  1.0,  1.0, -1.0, 0.0,  0.0],
        [  0.0,  0.0,  1.0, -1.0, -1.0,  1.0, 0.0,  0.0],
    ]),
   np.array([
        [  6.7,  6.6,  2.7,  2.7,  0.0,  0.0, 6.6,  6.6],
        [   RG,   RG,   RG,   RG,  0.0,  0.0, 0.0,  0.0],
        [  0.0,  0.0,   RG,   RG,   RG,   RG, 0.0,  0.0],
        [  0.0,  0.0,   RG,   RG,   RG,   RG, 0.0,  0.0],
    ])
)

config_slides_S = (
    np.array([
                    [ -1.0,   1.0,  1.0, 1.0],
                    [  0.0,  -1.0,  1.0, 1.0],
                    [  0.0,   0.0, -1.0, 1.0],
                    [  0.0,   0.0,  0.0, 0.0],
]),
    np.array([
                        [  1.0,   1.0,  1.0, 1.0],
                        [  0.0,   1.0,  1.0, 1.0],
                        [  0.0,   0.0,  1.0, 1.0],
                        [  0.0,   0.0,  0.0, 0.0],
]))

CONFIG_DIP_PIP_NO_INT = RouteConfig(
    D4=config_v1_no_internal[0],
    S4=config_v1_no_internal[1],
    desc="Original tendon routing plan containing the DIP/PIP flexor/extensor pair modelled without the internal tendon.",
    name="CONFIG_DIP_PIP_NO_INT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "PIP_ext",
        "MCP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_DIP_PIP_INT = RouteConfig(
    D4=config_v1_internal[0],
    S4=config_v1_internal[1],
    desc="Original tendon routing plan containing the DIP/PIP flexor/extensor pair modelled with the internal tendon.",
    name="CONFIG_DIP_PIP_INT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "INTERNAL",
        "PIP_ext",
        "MCP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_DIP_PAIR_NO_INT = RouteConfig(
    D4=config_v2_no_internal[0],
    S4=config_v2_no_internal[1],
    desc=f"New tendon routing idea containing tendon pair terminating both at DIP modelled without the internal tendon.\n (PIP extensor now routes over pulley (r=8, CCW+) on shaft 3, terminating distally).",
    name="CONFIG_DIP_PAIR_NO_INT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "DIP_ext",
        "MCP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_DIP_PAIR_INT = RouteConfig(
    D4=config_v2_internal[0],
    S4=config_v2_internal[1],
    desc=f"New tendon routing idea containing tendon pair terminating both at DIP modelled with the internal tendon.\n (PIP extensor now routes over pulley (r=8, CCW+) on shaft 3, terminating distally).",
    name="CONFIG_DIP_PAIR_INT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "INTERNAL",
        "DIP_ext",
        "MCP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_NO_MCP_EXT = RouteConfig(
    D4=config_v3_internal[0],
    S4=config_v3_internal[1],
    desc=f"New tendon routing idea for n+1 config. Flexor/extensor pair terminates at DIP\n MCP extensor tendon removed.",
    name="CONFIG_NO_MCP_EXT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "INTERNAL",
        "DIP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_NO_MCP_EXT_NO_INT = RouteConfig(
    D4=config_v3_no_internal[0],
    S4=config_v3_no_internal[1],
    desc=f"New tendon routing idea for n+1 config. Flexor/extensor pair terminates at DIP\n MCP extensor and internal tendon removed.",
    name="CONFIG_NO_MCP_EXT_NO_INT",
    tendon_names=[
        "MCP_flx",
        "DIP_flx",
        "DIP_ext",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_OPTION_1 = RouteConfig(
    D4=config_v4[0],
    S4=config_v4[1],
    desc=f"Julia's Option 1: 2 extensors, 1 flexor, and internal tendon.",
    name="CONFIG_OPTION_1",
    tendon_names=[
        "DIP_flx",
        "MCP_ext",
        "PIP_ext",
        "INTERNAL",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_OPTION_2 = RouteConfig(
    D4=config_v5[0],
    S4=config_v5[1],
    desc=f"Julia's Option 2: 1 extensor, 2 flexors, and internal tendon.",
    name="CONFIG_OPTION_2",
    tendon_names=[
        "DIP_ext",
        "MCP_flx",
        "PIP_flx",
        "INTERNAL",
        "SPLAY_A",
        "SPLAY_B",
    ],
)

CONFIG_OPTION_3A = RouteConfig(
    D4=config_v6[0],
    S4=config_v6[1],
    desc=f"Julia's Option 3: 2N tendon pairs at MCP, PIP, and internal.",
    name="CONFIG_OPTION_3A",
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

SLIDE_CONFIG = RouteConfig(
    D4=config_slides_S[1],
    S4=config_slides_S[0],
    desc="",
    name="test"
)

CONFIG_LIST = [CONFIG_DIP_PIP_NO_INT, CONFIG_DIP_PIP_INT, CONFIG_DIP_PAIR_NO_INT, CONFIG_DIP_PAIR_INT, CONFIG_NO_MCP_EXT, CONFIG_NO_MCP_EXT_NO_INT]
CONFIG_LIST2 = [CONFIG_OPTION_1, CONFIG_OPTION_2, CONFIG_OPTION_3A]
CONFIG_LIST.extend(CONFIG_LIST2)



