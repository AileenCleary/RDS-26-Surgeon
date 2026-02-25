from __future__ import annotations

import numpy as np

def nnls_projected_gradient(A: np.ndarray, b: np.ndarray, iters: int = 5000) -> np.ndarray:
    A = np.asarray(A, float)
    b = np.asarray(b, float).reshape(-1)

    m, n = A.shape
    x = np.zeros(n, float)

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


def nnls(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    try:
        from scipy.optimize import nnls as scipy_nnls  # type: ignore
        x, _ = scipy_nnls(np.asarray(A, float), np.asarray(b, float).reshape(-1))
        return np.asarray(x, float)
    except Exception:
        return nnls_projected_gradient(A, b)