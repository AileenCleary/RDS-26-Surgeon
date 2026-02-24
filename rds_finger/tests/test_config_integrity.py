import numpy as np
from rds_finger.model import FingerModel
from rds_finger import config

def test_config_resolves_world():
    m = FingerModel(
        link_lengths=config.LINK_LENGTHS,
        coupling_ratio=config.COUPLING_RATIO,
        shafts=config.SHAFTS,
        pulleys=config.PULLEYS,
        drums=config.DRUMS,
        endpoints=config.ENDPOINTS,
        tendons=config.TENDONS,
        tendon_order=config.TENDON_ORDER,
        bearings=config.BEARINGS,
    )
    frames, tip_pose, wpulleys, wendpoints, wdrums, wshafts, wbearing = m.world_state(np.array([0.0,0.1,0.2]))
    assert "pulley_01" in wpulleys
    assert "mcp_ext_end" in wendpoints
    assert "drum_MCP_EXT" in wdrums
    