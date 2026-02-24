import numpy as np
from rds_finger.kinematics.chain import compute_frames

def test_frames_move_distal():
    link_lengths = dict(splay=10.0, proximal=20.0, middle=30.0, distal=40.0)
    frames0 = compute_frames(np.array([0,0,0]), link_lengths=link_lengths, coupling_ratio=0.7)
    frames1 = compute_frames(np.array([0,0.1,0]), link_lengths=link_lengths, coupling_ratio=0.7)
    # O2 position should change when MCP changes
    assert not np.allclose(frames0["O2"].p, frames1["O2"].p)