from __future__ import annotations

import numpy as np

def nnls(A: np.ndarray, b: np.ndarray) -> np.ndarray:
    A = np.asarray(A, float)
    b = np.asarray(b, float).reshape(-1)
    try:
        from scipy.optimize import nnls as scipy_nnls  
        x, _ = scipy_nnls(A, b)
        return x
    except Exception:
        x = np.zeros(A.shape[1], float)
        AtA = A.T @ A
        Atb = A.T @ b
        L = float(np.linalg.norm(AtA, 2))
        if L < 1e-12:
            return x
        alpha = 1.0 / L
        for _ in range(6000):
            g = AtA @ x - Atb
            x2 = x - alpha * g
            x2[x2 < 0] = 0.0
            if np.linalg.norm(x2 - x) < 1e-10:
                x = x2
                break
            x = x2
        return x

def nnls_with_preload(A, tau, preload_idx: int | None = None, preload: float = 0.0, w: float = 1e-3):
    A = np.asarray(A, float)
    tau = np.asarray(tau, float).reshape(-1)
    m = A.shape[1]
    T0 = np.zeros(m, float)
    if preload_idx is not None:
        T0[preload_idx] = float(preload)
    W = np.sqrt(float(w))
    A_aug = np.vstack([A, W*np.eye(m)])
    b_aug = np.concatenate([tau, W*T0])
    return nnls(A_aug, b_aug)