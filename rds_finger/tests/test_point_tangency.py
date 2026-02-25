import numpy as np
from rds_finger.routing.tangency_point import tangents_point_circle_parallel_axis
from rds_finger.core.math3d import unit

def test_point_circle_tangency():
    c = np.array([0.0,0.0,0.0])
    r = 5.0
    axis = np.array([0.0,1.0,0.0])
    p = np.array([20.0,0.0,0.0])

    tpts = tangents_point_circle_parallel_axis(p, c, r, axis)
    assert len(tpts) == 2

    for q in tpts:
        assert abs(np.linalg.norm(q - c) - r) < 1e-6

        u = q - c
        v = p - q
        a = unit(axis)
        u = u - np.dot(u,a)*a
        v = v - np.dot(v,a)*a
        assert abs(np.dot(u, v)) < 1e-6