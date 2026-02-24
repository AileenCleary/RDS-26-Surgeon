import numpy as np
from rds_finger.analysis.solvers.solve import nnls

def test_nnls_simple():
    A = np.array([[1.0, 0.0],
                  [0.0, 1.0]])
    b = np.array([2.0, 3.0])
    x = nnls(A, b)
    assert np.allclose(x, b)
    assert np.all(x >= 0)