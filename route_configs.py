import numpy as np

config_v1_no_internal = (
    # mcp flx, dip flx, pip ext, mcp ext, splay A/B
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
    # mcp flx, dip flx, internal, pip ext, mcp ext, splay A/B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  0.0,  0.0,  0.0,  0.0,  0.0],
    ]),
    np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  0.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  0.0,  0.0,  0.0,  0.0,  0.0],
    ])
)

config_v2_no_internal = (
    # mcp flx, dip flx, dip ext, mcp ext, splay A/B
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
    # mcp flx, dip flx, internal, dip ext, mcp ext, splay A/B
    np.array([
        [ -1.0,  1.0,  0.0, -1.0,  1.0,  1.0, -1.0],
        [ -1.0,  1.0,  0.0,  1.0,  1.0,  0.0,  0.0],
        [  0.0, -1.0, -1.0,  1.0,  0.0,  0.0,  0.0],
        [  0.0, -1.0,  0.0,  1.0,  0.0,  0.0,  0.0],
    ]),
   np.array([
        [  6.6,  2.7,  0.0,  2.7,  6.6,  6.6,  6.6],
        [  8.0,  5.5,  0.0,  5.0,  8.0,  0.0,  0.0],
        [  0.0,  5.0,  5.0,  8.0,  0.0,  0.0,  0.0],
        [  0.0,  8.0,  0.0,  8.0,  0.0,  0.0,  0.0],
    ])
)

def generalized_S(D4, S4):
    S3 = S4[:3, :]
    D3 = D4[:3, :]

    theta_dip = 0.7 * S4[3, :] * D4[3, :]

    S = S3 * D3
    S[2, :] = S[2, :] + theta_dip

    return S

class RouteConfig():

    def __init__(
            self,
            D4,
            S4,
            desc,
            name,
    ):
        
        self.D4 = D4
        self.S4 = S4
        self.desc = desc
        self.name = name
        self.S = generalized_S(D4, S4)

CONFIG_DIP_PIP_NO_INT = RouteConfig(
    D4=config_v1_no_internal[0],
    S4=config_v1_no_internal[1],
    desc="Original tendon routing plan containing the DIP/PIP flexor/extensor pair modelled without the internal tendon.",
    name="CONFIG_DIP_PIP_NO_INT",
)

CONFIG_DIP_PIP_INT = RouteConfig(
    D4=config_v1_internal[0],
    S4=config_v1_internal[1],
    desc="Original tendon routing plan containing the DIP/PIP flexor/extensor pair modelled with the internal tendon.",
    name="CONFIG_DIP_PIP_INT",
)

CONFIG_DIP_PAIR_NO_INT = RouteConfig(
    D4=config_v2_no_internal[0],
    S4=config_v2_no_internal[1],
    desc=f"New tendon routing idea containing tendon pair terminating both at DIP modelled without the internal tendon.\n (PIP extensor now routes over pulley (r=8, CCW+) on shaft 3, terminating distally).",
    name="CONFIG_DIP_PAIR_NO_INT",
)

CONFIG_DIP_PAIR_INT = RouteConfig(
    D4=config_v2_internal[0],
    S4=config_v2_internal[1],
    desc=f"New tendon routing idea containing tendon pair terminating both at DIP modelled with the internal tendon.\n (PIP extensor now routes over pulley (r=8, CCW+) on shaft 3, terminating distally).",
    name="CONFIG_DIP_PAIR_INT",
)

CONFIG_LIST = [CONFIG_DIP_PIP_NO_INT, CONFIG_DIP_PIP_INT, CONFIG_DIP_PAIR_NO_INT, CONFIG_DIP_PAIR_INT]



