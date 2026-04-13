import modern_robotics as md
import numpy as np
from rds_finger.tensionability.route_configs import CONFIG
from scipy.optimize import linprog

"""
GEOMETRY, ASSUMPTIONS, AND SIMPLIFICATIONS

1. The world coordinate frame defines +x as moving from proximal to distal along the finger, +z moving from
    palmar/ventral to dorsal, and +y as moving from left to right when viewing from top-down.
2. The world origin x and y are the center of the splay shaft (0), and the z is on the plane bisecting the
    finger, parallel to the dorsal/palmar side of the finger, i.e., sharing the same z as shafts 1, 2, and 3.
3. Counterclockwise (CCW) rotation about an axis is positive.
4. Metric units (mm, N).
5. "Home" or "default" position of the finger is the finger completely extended, pointing straight ahead.
6. Assuming tendons terminate 10 mm down the proceeding link/phalanx. Tension in = Tension out, ignoring
    friction and dynamic effects.
7. Force from the pulleys only transmitted if they cannot freely rotate/rotate with the shaft.
8. Assuming a default pulley radius, shaft diameter and length, bearing and pulley positions along the shaft.
"""

# ============================================ CONSTANTS =======================================================

# DIP is mechanically coupled to PIP and is not independently actuated.
# It is excluded from Slist; its torque contribution is accounted for via the coupling ratio.
JOINT_NAMES_ORDERED    = ["SPLAY", "MCP", "PIP"]  # DIP coupled to PIP via DEFINED_COUPLING_RATIO
DEFINED_COUPLING_RATIO = 0.7   # tau_dip = DEFINED_COUPLING_RATIO * tau_pip

MOTOR_PULLEY_RADIUS    = 4     # mm
MOTOR_OPTIONS          = [["DS42S01", 950.0]]   # [name, stall torque (N*mm)]
MOTOR_OPTIONS_TENSIONS = sorted(
    [[name, torque / MOTOR_PULLEY_RADIUS] for name, torque in MOTOR_OPTIONS],
    key=lambda x: x[1],
)

PRELOAD     = 1                          # N — desired baseline tension for all tendons
MAX_TENSION = MOTOR_OPTIONS_TENSIONS[0][1]  # N — motor tension limit, used in both solvers

TEST_MOTOR_OPTIONS_VERBOSE = True

np.set_printoptions(
    precision=2,
    suppress=True,
    linewidth=120,
)

# ============================================ LINK GEOMETRY ===================================================

L_splay    = 21   # mm
L_proximal = 44   # mm
L_middle   = 39   # mm
L_distal   = 34   # mm

z_hat = np.array([0.0, 0.0, 1.0])
y_hat = np.array([0.0, 1.0, 0.0])

x_mcp = L_splay
x_pip = L_splay + L_proximal
x_dip = L_splay + L_proximal + L_middle

q_splay = np.array([0.0,   0.0, 0.0])
q_mcp   = np.array([x_mcp, 0.0, 0.0])
q_pip   = np.array([x_pip, 0.0, 0.0])
q_dip   = np.array([x_dip, 0.0, 0.0])

# ============================================ JACOBIAN & JOINT TORQUES ========================================

def screw_axis(omega, q):
    """Compute the space screw axis given rotation axis omega and point q on the axis."""
    v = -np.cross(omega, q)
    return np.r_[omega, v]

S_splay = screw_axis(z_hat, q_splay)
S_mcp   = screw_axis(y_hat, q_mcp)
S_pip   = screw_axis(y_hat, q_pip)
S_dip   = screw_axis(y_hat, q_dip)  # defined but excluded from Slist (DIP is coupled)

# DIP excluded: its torque requirement is handled by the passive coupling mechanism.
Slist = np.column_stack([S_splay, S_mcp, S_pip])
thetalist_home = np.zeros(3)

Js = md.JacobianSpace(Slist=Slist, thetalist=thetalist_home)
print(f"\nSpace Jacobian Js:\n{Js}")

# Tip force: 20 N in +z (palmar-to-dorsal) applied at the center of the distal phalanx.
# Spatial wrench convention (Modern Robotics): Fs = [moment, force]
f_tip       = np.array([0.0, 0.0, 20.0])
x_tip_force = x_dip + 0.5 * L_distal
r_tip       = np.array([x_tip_force, 0.0, 0.0])
m_tip       = np.cross(r_tip, f_tip)
Fs_tip      = np.r_[m_tip, f_tip]   # [mx, my, mz, fx, fy, fz]

# Joint torques from virtual work: tau = Js^T * Fs
tau_joint = Js.T @ Fs_tip

print(f"\nJoint Torques (N*mm):")
for name, torque in zip(JOINT_NAMES_ORDERED, tau_joint):
    print(f"    {name}: {torque:.4f}")

# ============================================ ROUTING MATRIX SETUP ============================================

def merge_splay_columns(S, tendon_names, splay_A_idx=6, splay_B_idx=7):
    """
    Merge SPLAY_A and SPLAY_B into a single column representing the single
    shared-shaft splay actuator.

    Physical rationale:
        SPLAY_A and SPLAY_B are wound in opposite directions on the same motor
        shaft. When the motor drives with tension T_motor, both tendons carry
        that tension simultaneously — one is pulled in while the other pays out.
        Their signs (+1 and -1) are already encoded in S (S = D * moment_arms),
        so the net routing column is:

            S_net = S[:,splay_A] + S[:,splay_B]

        This correctly yields zero net torque at the splay joint under equal
        tension (since r11 + (-r11) = 0), while preserving any cross-joint
        contributions from tendon routing over other pulleys.

    Parameters
    ----------
    S            : (m, n) routing matrix with signs encoded
    tendon_names : list of n tendon name strings
    splay_A_idx  : column index of SPLAY_A (default 6)
    splay_B_idx  : column index of SPLAY_B (default 7)

    Returns
    -------
    S_merged     : (m, n-1) merged routing matrix
    names_merged : list of n-1 tendon name strings
    """
    S_merged = np.delete(S, splay_B_idx, axis=1)
    S_merged[:, splay_A_idx] = S[:, splay_A_idx] + S[:, splay_B_idx]
    names_merged = [name for i, name in enumerate(tendon_names) if i != splay_B_idx]
    names_merged[splay_A_idx] = "SPLAY"
    return S_merged, names_merged


def identify_active_tendons(S):
    """
    Partition tendon columns into 'active' (at least one non-zero entry — the
    tendon drives a joint via a driver pulley) and 'free' (all-zero column —
    all pulleys on this tendon's path are idlers, or the joint is not in the
    current model).

    Free tendons are pinned to PRELOAD and excluded from all solvers.

    Returns
    -------
    active_idx : list[int]  column indices with at least one non-zero entry
    free_idx   : list[int]  column indices that are entirely zero
    """
    active_idx = [i for i in range(S.shape[1]) if np.any(S[:, i] != 0.0)]
    free_idx   = [i for i in range(S.shape[1]) if np.all(S[:, i] == 0.0)]
    return active_idx, free_idx


# Build the full merged routing matrix
S, TENDON_NAMES_ORDERED = merge_splay_columns(CONFIG.S, CONFIG.tendon_names)

# Split into active and free subspaces
ACTIVE_IDX, FREE_IDX = identify_active_tendons(S)
S_active     = S[:, ACTIVE_IDX]
ACTIVE_NAMES = [TENDON_NAMES_ORDERED[i] for i in ACTIVE_IDX]
FREE_NAMES   = [TENDON_NAMES_ORDERED[i] for i in FREE_IDX]

print(f"\nRouting matrix S ({S.shape[0]} joints x {S.shape[1]} tendons):\n{S}")
print(f"\nTendon names:  {TENDON_NAMES_ORDERED}")
print(f"Active tendons (driver pulleys, contribute torque): {ACTIVE_NAMES}")
print(f"Free tendons   (all idlers, pinned to preload):     {FREE_NAMES}")

# --- Feasibility check: is tau_joint reachable by the active subspace? ---
rank             = np.linalg.matrix_rank(S_active)
T_lstsq, _, _, _ = np.linalg.lstsq(S_active, tau_joint, rcond=None)
lstsq_residual   = np.linalg.norm(S_active @ T_lstsq - tau_joint)
feasible         = lstsq_residual < 1e-6
print(f"\nActive S shape: {S_active.shape}, rank: {rank}")
print(f"Least-squares T (no sign constraint): {T_lstsq}")
print(f"Least-squares residual: {lstsq_residual:.4e}"
      + (" ✓ (tau reachable)" if feasible else " ✗ (tau NOT in column space — check routing matrix)"))
if not feasible:
    raise RuntimeError(
        "tau_joint is not in the column space of S_active. "
        "Check the routing matrix for incorrect idler/driver assignments or wrong signs."
    )

# ============================================ SOLVERS =========================================================

def solve_antagonist_pairs_preload(S_active, tau, active_names, preload, max_tension):
    """
    Closed-form minimum-effort solver for decoupled antagonist tendon pairs.

    For each joint row that has exactly one flexor and one extensor (a standard
    2N antagonist pair), the minimum-effort solution is found analytically:
      - Set the antagonist (the tendon working against the torque demand) to
        the preload baseline tension.
      - Solve for the agonist (the tendon working with the torque demand) from
        the single scalar torque equation.
      - If the agonist would exceed max_tension, clamp it and raise the
        antagonist to compensate.

    This is exact and requires no iteration, because the current routing matrix
    is block-diagonal (MCP and PIP rows are fully decoupled). If the routing
    ever becomes coupled (e.g. tendons spanning multiple joints with driver
    pulleys), the fallback least-squares path in the else-branch handles it.

    Why not SLSQP?
        With rank-2, 4-tendon active S, SLSQP faces a 2D null space and
        consistently returns the warm-start unchanged (singular Hessian in the
        LSQ subproblem). The closed-form approach here is both faster and
        exactly correct for the block-diagonal case.

    Parameters
    ----------
    S_active     : (m, n_active) routing matrix, active tendons only
    tau          : (m,) joint torque vector
    active_names : list of n_active tendon name strings
    preload      : float, minimum (antagonist) tension
    max_tension  : float, motor upper limit

    Returns
    -------
    T_active : (n_active,) tension vector for active tendons
    info     : dict  keyed by joint index, with per-joint diagnostics
    """
    m, n    = S_active.shape
    T_active = np.zeros(n, dtype=float)
    info     = {}

    for j in range(m):
        row      = S_active[j, :]
        nonzero  = np.where(row != 0.0)[0]

        if len(nonzero) == 0:
            # No active tendons at this joint — tau[j] must be 0 (verified above)
            continue

        if len(nonzero) == 2:
            # ---- Standard antagonist pair: one agonist, one antagonist ----
            i_a, i_b = nonzero
            r_a, r_b = row[i_a], row[i_b]

            # Agonist: whichever tendon produces torque in the same direction
            # as tau[j] when tensioned (i.e., r * T adds to tau in sign)
            if r_a * tau[j] >= 0:
                i_ago, r_ago = i_a, r_a
                i_ant, r_ant = i_b, r_b
            else:
                i_ago, r_ago = i_b, r_b
                i_ant, r_ant = i_a, r_a

            # Minimum effort: antagonist at preload, agonist solves the equation
            T_ant = preload
            T_ago = (tau[j] - r_ant * T_ant) / r_ago

            # Clamp agonist to motor limit; redistribute excess to antagonist
            if T_ago > max_tension:
                T_ago = max_tension
                T_ant = (tau[j] - r_ago * T_ago) / r_ant
                if T_ant < preload:
                    # Motor is genuinely undersized for this torque demand
                    T_ant = preload
                    print(
                        f"WARNING: joint {j} "
                        f"({JOINT_NAMES_ORDERED[j] if j < len(JOINT_NAMES_ORDERED) else j}): "
                        f"torque demand {tau[j]:.2f} N*mm exceeds motor capability "
                        f"at max tension {max_tension:.2f} N. Clamping to best achievable."
                    )

            # Enforce preload floor on agonist too (handles near-zero tau)
            T_ago = max(T_ago, preload)

            T_active[i_ago] = T_ago
            T_active[i_ant] = T_ant

            info[j] = {
                "agonist":      active_names[i_ago],
                "antagonist":   active_names[i_ant],
                "T_agonist":    T_ago,
                "T_antagonist": T_ant,
                "tau_achieved": r_ago * T_ago + r_ant * T_ant,
            }

        else:
            # ---- Coupled case: >2 tendons share this joint row ----
            # Solve for the delta above the preload baseline via least-squares,
            # then clip to [preload, max_tension].
            T_base  = np.full(n, preload, dtype=float)
            tau_res = tau - S_active @ T_base
            dT, _, _, _ = np.linalg.lstsq(S_active, tau_res, rcond=None)
            T_active    = np.clip(T_base + dT, preload, max_tension)
            info[j]     = {
                "note": (
                    f"joint {j} has {len(nonzero)} active tendons — "
                    "used least-squares fallback (coupled routing)"
                )
            }

    return T_active, info


def solve_tendon_tensions_maxmin(S_active, tau, motor_options_tensions, verbose=True):
    """
    Solve tendon tensions over the active subspace by maximizing the minimum
    tension (max-min LP formulation). Free tendons are handled separately at
    preload by the caller.

    This finds the solution where the least-loaded active tendon is as loaded
    as possible — maximizing the slack margin across all active tendons.

    Formulation (LP):
        Maximize  t*
        Subject to:
            S_active @ T = tau
            T_i >= t*             (no active tendon goes below t*)
            0 <= T_i <= max_tension

    Note: for the current block-diagonal S_active, the max-min solution spreads
    load evenly across each antagonist pair. With a higher motor limit the LP
    will raise both tendons in each pair proportionally since there is no cost
    to adding equal co-contraction (it lives in the null space).

    Parameters
    ----------
    S_active               : (m, n_active) routing matrix, active tendons only
    tau                    : (m,) joint torque vector
    motor_options_tensions : list of [name, max_tension] sorted by max_tension
    verbose                : if True, try all motors; if False, use median motor

    Returns
    -------
    results : list of (message_str, OptimizeResult) tuples for successful motors
    """
    S   = np.asarray(S_active, dtype=float)
    tau = np.asarray(tau,      dtype=float)
    m, n = S.shape

    # LP variable layout: [T_0, ..., T_{n-1}, t*]
    c     = np.zeros(n + 1)
    c[-1] = -1.0   # minimize -t*  =>  maximize t*

    A_eq        = np.zeros((m, n + 1))
    A_eq[:, :n] = S
    b_eq        = tau

    # -T_i + t* <= 0  =>  T_i >= t*
    A_ub = np.zeros((n, n + 1))
    b_ub = np.zeros(n)
    for i in range(n):
        A_ub[i, i]  = -1.0
        A_ub[i, -1] =  1.0

    results        = []
    options_to_try = (motor_options_tensions if verbose
                      else [motor_options_tensions[len(motor_options_tensions) // 2]])

    for name, max_tension in options_to_try:
        bounds = [(0.0, max_tension)] * n + [(None, None)]
        res = linprog(c, A_ub, b_ub, A_eq, b_eq, bounds, method="highs")
        if res.success:
            results.append((f"\nMotor: {name} | Max Tension: {max_tension:.2f} N\n    Success!", res))
        else:
            print(f"\nMotor: {name} | Max Tension: {max_tension:.2f} N\n    Failed: {res.message}")

    return results


def reconstruct_full_tensions(T_active, active_idx, n_total, preload):
    """
    Reconstruct the full tension vector from the active-subspace solution,
    pinning free (zero-column) tendons to preload.

    Parameters
    ----------
    T_active   : (n_active,) tensions for active tendons
    active_idx : list of active tendon indices in the full vector
    n_total    : total number of tendons (active + free)
    preload    : tension assigned to free tendons

    Returns
    -------
    T_full : (n_total,) complete tension vector
    """
    T_full = np.full(n_total, preload, dtype=float)
    for i, idx in enumerate(active_idx):
        T_full[idx] = T_active[i]
    return T_full


# ============================================ SOLVE & PRINT ===================================================

n_total = len(TENDON_NAMES_ORDERED)

# -----------------------------------------------------------------------
# Solver 1: Closed-form preload solver (minimum effort, antagonist pairs)
#
# For each antagonist pair: antagonist = PRELOAD, agonist solves the
# torque equation. This is the minimum co-contraction solution — the
# physically correct answer for "what tensions are needed and no more".
# -----------------------------------------------------------------------
T_active_preload, pair_info = solve_antagonist_pairs_preload(
    S_active, tau_joint,
    active_names=ACTIVE_NAMES,
    preload=PRELOAD,
    max_tension=161,
)
T_full_preload  = reconstruct_full_tensions(T_active_preload, ACTIVE_IDX, n_total, PRELOAD)
tau_hat_preload = S @ T_full_preload

print(f"\n{'='*60}")
print(f"Preload Solution (minimum effort — antagonist = preload):")
print(f"{'='*60}")
print(f"\nTendon Tensions (N):")
for name, tension in zip(TENDON_NAMES_ORDERED, T_full_preload):
    tag = "  [free — pinned to preload]" if name in FREE_NAMES else ""
    print(f"    {name}: {tension:.4f}{tag}")
print(f"\nPer-joint breakdown:")
for j, d in pair_info.items():
    if "note" in d:
        print(f"    Joint {j}: {d['note']}")
    else:
        jname = JOINT_NAMES_ORDERED[j] if j < len(JOINT_NAMES_ORDERED) else f"joint_{j}"
        print(f"    {jname}: "
              f"agonist={d['agonist']} ({d['T_agonist']:.2f} N)  "
              f"antagonist={d['antagonist']} ({d['T_antagonist']:.2f} N)  "
              f"tau_achieved={d['tau_achieved']:.2f} N*mm")
print(f"\ntau_hat:       {tau_hat_preload}")
print(f"tau_joint:     {tau_joint}")
print(f"residual norm: {np.linalg.norm(tau_hat_preload - tau_joint):.4e}")

# -----------------------------------------------------------------------
# Solver 2: Max-min LP (maximize minimum active tendon tension)
#
# Finds the solution that maximizes the slack margin across all active
# tendons. For the block-diagonal S this raises co-contraction in each
# pair until the motor limit or t* ceiling is hit. Useful for checking
# whether the chosen motor can handle the required torques with margin.
# -----------------------------------------------------------------------
results_maxmin = solve_tendon_tensions_maxmin(
    S_active, tau_joint, MOTOR_OPTIONS_TENSIONS, verbose=TEST_MOTOR_OPTIONS_VERBOSE
)

print(f"\n{'='*60}")
print(f"Max-Min LP Solution (maximize minimum active tendon tension):")
print(f"{'='*60}")

for msg, res in results_maxmin:
    T_active_mm = res.x[:-1]
    t_star      = res.x[-1]
    T_full_mm   = reconstruct_full_tensions(T_active_mm, ACTIVE_IDX, n_total, PRELOAD)
    tau_hat_mm  = S @ T_full_mm
    print(msg)
    print(f"\nTendon Tensions (N):")
    for name, tension in zip(TENDON_NAMES_ORDERED, T_full_mm):
        tag = "  [free — pinned to preload]" if name in FREE_NAMES else ""
        print(f"    {name}: {tension:.4f}{tag}")
    print(f"\nt* (min active tension): {t_star:.4f} N")
    print(f"tau_hat:                 {tau_hat_mm}")
    print(f"tau_joint:               {tau_joint}")
    print(f"residual norm:           {np.linalg.norm(tau_hat_mm - tau_joint):.4e}")

# ============================================ SHAFT / TERMINATION =============================================

SHAFT_LENGTH_DEFAULT = 30   # mm — used in shaft geometry calculations (see route_configs)
TERMINATION_OFFSET   = 10   # mm — tendon termination offset into proceeding phalanx (see assumption 6)