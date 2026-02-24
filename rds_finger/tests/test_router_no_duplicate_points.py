import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config
from rds_finger.routing.router import route_tendons

def build_model():
    return FingerModel(
        link_lengths=config.LINK_LENGTHS,
        coupling_ratio=config.COUPLING_RATIO,
        shafts=config.SHAFTS,
        pulleys=config.PULLEYS,
        drums=config.DRUMS,
        endpoints=config.ENDPOINTS,
        tendons=config.TENDONS,
        tendon_order=config.TENDON_ORDER,
        bearings=config.BEARINGS
    )

def test_no_zero_length_segments():
    m = build_model()
    frames, tip_pose, wp, we, wd, wshafts, wbearing = m.world_state(np.array([0.0, 0.3, 0.5]))

    rt = route_tendons(m.tendons["MCP_EXT"], wp, we, wd)
    pts = rt.points

    # no consecutive duplicate points
    for i in range(len(pts)-1):
        assert np.linalg.norm(pts[i+1] - pts[i]) > 1e-9