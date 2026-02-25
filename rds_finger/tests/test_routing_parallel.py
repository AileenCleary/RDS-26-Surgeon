import numpy as np
from rds_finger.routing.tangency_parallel import tangents_two_circles_parallel_axis

def test_parallel_tangents_count():
    c1 = np.array([0.0,0.0,0.0])
    c2 = np.array([20.0,0.0,0.0])
    axis = np.array([0.0,1.0,0.0])
    tans = tangents_two_circles_parallel_axis(c1, 5.0, c2, 3.0, axis)
    assert len(tans) == 4
    for p1,p2 in tans:
        assert abs(np.linalg.norm(p1-c1) - 5.0) < 1e-6