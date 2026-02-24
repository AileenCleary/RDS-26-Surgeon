import numpy as np
from rds_finger.core.frames import Pose
from rds_finger.core.math3d import rot_axis_angle

def test_pose_inverse():
    R = rot_axis_angle([0,1,0], 0.3)
    p = np.array([1.0,2.0,3.0])
    A = Pose(R,p)
    I = A @ A.inv()
    assert(np.allclose(I.R, np.eye(3), atol=1e-9))
    assert(np.allclose(I.p, np.zeros(3), atol=1e-9))