"""
static_solver.py — 静力学求解器
给定当前角度，求解对抗重力所需的最小 tendon 张力，反推 L_slack
"""

import numpy as np
from scipy.optimize import linprog

# 关节名称到 q 向量索引的映射
JOINT_IDX = {"mcp_splay": 0, "mcp_flexion": 1, "pip": 2, "dip": 3}


def solve_tendon_forces(tau_gravity, tendons):
    """
    求解最小 tendon 张力，使得 tendon 力矩对抗重力力矩。

    Drake 的 CalcGravityGeneralizedForces 返回的是重力产生的广义力，
    符号规定：tau_gravity 加上 tendon 力矩 = 0（静力平衡）
    即：tau_tendon = -tau_gravity

    每根 tendon 对关节 j 的力矩：tau_i = sign_i * r_i * F_i
    约束：F_i >= 0（只能拉）
    目标：min sum(F_i)
    """
    n_tendons = len(tendons)
    n_joints  = 3   # splay(0), mcp_flexion(1), pip(2)，dip 由 coupling 决定不独立

    # 构建力矩矩阵 A[j, i] = sign_i * r_i
    A = np.zeros((n_joints, n_tendons))
    for i, t in enumerate(tendons):
        j = JOINT_IDX.get(t["joint"], -1)
        if 0 <= j < n_joints:
            A[j, i] = t["sign"] * t["pulley_r"]

    # 平衡条件：A @ F = -tau_gravity[:3]
    tau_combined = tau_gravity[:3].copy()
    tau_combined[2] += (2.0 / 3.0) * tau_gravity[3]   # 0.7 是 ALPHA_DIP_PIP
    tau_target = -tau_combined

    # 线性规划：min sum(F), F >= 0, A@F = tau_target
    c      = np.ones(n_tendons)
    bounds = [(0, None)] * n_tendons

    result = linprog(c, A_eq=A, b_eq=tau_target, bounds=bounds, method='highs')

    if result.success:
        return result.x
    else:
        print(f"WARNING 静力学求解失败: {result.message}")
        # 退而求其次用伪逆，clamp 负值
        F = np.linalg.lstsq(A, tau_target, rcond=None)[0]
        return np.maximum(F, 0.0)


def forces_to_lslack(F_target, tendons, q_current, k):
    """
    给定目标张力 F_i，反推 L_slack_i。
    """
    L_slacks = np.zeros(len(tendons))
    for i, t in enumerate(tendons):
        idx   = JOINT_IDX[t["joint"]]
        theta = q_current[idx]
        L     = t["L_ref"] - t["sign"] * t["pulley_r"] * (theta - t["theta_ref"])
        # 这里用传入的 k
        L_slacks[i] = max(0.001, L - F_target[i] / k)
    return L_slacks
