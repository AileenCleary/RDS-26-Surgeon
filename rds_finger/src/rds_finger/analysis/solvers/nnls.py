from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


def nnls_projected_gradient(A: NDArray[np.float64], b: NDArray[np.float64], iters: int = 5000) -> NDArray[np.float64]:
    """
    Simple NNLS fallback: projected gradient descent on 0.5||Ax-b||^2 with x>=0.
    Not the fastest, but robust and dependency-free.

    Returns x >= 0.
    """
    A = np.asarray(A, float)
    b = np.asarray(b, float).reshape(-1)

    m, n = A.shape
    x = np.zeros(n, float)

    # Lipschitz constant for grad: ||A^T A||_2. Approx via power iteration.
    def power_iter(M, k=50):
        v = np.random.randn(M.shape[1])
        v /= np.linalg.norm(v)
        for _ in range(k):
            v = M @ v
            nv = np.linalg.norm(v)
            if nv < 1e-12:
                break
            v /= nv
        return float(v @ (M @ v))

    ATA = A.T @ A
    L = power_iter(ATA)
    step = 1.0 / max(L, 1e-12)

    for _ in range(iters):
        grad = A.T @ (A @ x - b)
        x = x - step * grad
        x[x < 0.0] = 0.0

    return x


def nnls(A: NDArray[np.float64], b: NDArray[np.float64]) -> NDArray[np.float64]:
    """
    NNLS wrapper: use scipy.optimize.nnls if available, else fallback.
    """
    try:
        from scipy.optimize import nnls as scipy_nnls  # type: ignore
        x, _ = scipy_nnls(np.asarray(A, float), np.asarray(b, float).reshape(-1))
        return np.asarray(x, float)
    except Exception:
        return nnls_projected_gradient(A, b)