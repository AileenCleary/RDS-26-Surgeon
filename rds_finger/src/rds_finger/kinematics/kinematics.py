import modern_robotics as md
import numpy as np
from rds_finger.tensionability.route_configs import CONFIG
from scipy.optimize import nnls, linprog, minimize
"""
GEOMETRY, ASSUMPTIONS, AND SIMPLIFICATIONS

1. The world coordinate frame defines +x as moving from proximal to distal along the finger, +z moving from palmar/ventral to dorsal, and
    +y as moving from left to right when viewing from top-down.
2. The world origin x and y are the center of the splay shaft (0), and the z is on the plane bisecting the finger, parallel to the dorsal/palmar side of the finger,
    i.e., sharing the same z as shafts 1, 2, and 3.
3. Counterclockwise (CCW) rotation about an axis is positive.
4. Metric units (mm, N).
5. "Home" or "default" position of the finger is the finger completely extended, pointing straight ahead.
6. Assuming tendons terminate 10 mm down the proceeding link/phalanx. Tension in = Tension out, ignoring friction and dynamic effects. 
7. Force from the pulleys only transmitted if they cannot freely rotate/rotate with the shaft.
8. Assuming a default pulley radius, shaft diameter and length, bearing and pulley positions along the shaft.
"""
# LENGTH_DEFAULT = 25

JOINT_NAMES_ORDERED = ["SPLAY", "MCP", "PIP", "DIP"]
DEFINED_COUPLING_RATIO = 0.7

TENDON_NAMES_ORDERED = CONFIG.tendon_names

MOTOR_PULLEY_RADIUS = 4
# MOTOR_OPTIONS = [["DCX_22", 640], ["DCX_26", 653], ["ECX_FLAT_22", 625], ["ECX_TORQUE_22", 690.8], ["ECX FLAT 32", 691.5], ["EC-i_30", 1785.95]] #name, torque
MOTOR_OPTIONS = [["DS42S01", 950.0]]
MOTOR_OPTIONS_TENSIONS = sorted(list(map(lambda n: [n[0], n[1]/MOTOR_PULLEY_RADIUS], MOTOR_OPTIONS)), key=lambda x: x[1])

TEST_MOTOR_OPTIONS_VERBOSE = True

np.set_printoptions(
    precision=2,
    suppress=True,
    linewidth=120,
)

# Link lengths 
# L_splay = L_proximal = L_middle = LENGTH_DEFAULT; L_distal = 0.5*LENGTH_DEFAULT
L_splay = 21
L_proximal = 44
L_middle = 39
L_distal = 34

z_hat = np.array([0.0, 0.0, 1.0])
y_hat = np.array([0.0, 1.0, 0.0])

x_mcp= L_splay
x_pip = L_splay + L_proximal
x_dip = L_splay + L_proximal + L_middle

q_splay = np.array([0.0, 0.0, 0.0])
q_mcp = np.array([x_mcp, 0.0, 0.0])
q_pip = np.array([x_pip, 0.0, 0.0])
q_dip = np.array([x_dip, 0.0, 0.0])

def screw_axis(omega, q):
    v = -np.cross(omega, q)
    return np.r_[omega, v]

S_splay = screw_axis(z_hat, q_splay)
S_mcp = screw_axis(y_hat, q_mcp)
S_pip = screw_axis(y_hat, q_pip)
S_dip = screw_axis(y_hat, q_dip)

Slist = np.column_stack([S_splay, S_mcp, S_pip]) # S_dip
thetalist_home = np.array([0.0, 0.0, 0.0]) # 0.0

Js = md.JacobianSpace(Slist=Slist, thetalist=thetalist_home)
print(f"\nSpace Jacobian Js:\n{Js}")

f_tip = np.array([0.0, 0.0, 20.0])
x_tip_force = x_dip + 0.5*L_distal
r_tip = np.array([x_tip_force, 0.0, 0.0])
m_tip = np.cross(r_tip, f_tip)
Fs_tip = np.r_[m_tip, f_tip]

# JOINT TORQUES 
tau_joint = Js.T @ Fs_tip

print(f"\nJoint Torques (N mm):")
for idx, torque in enumerate(tau_joint):
    print(f"    {JOINT_NAMES_ORDERED[idx]}: {torque}    {'[where tau is dependent and coupled with PIP (r=0.7)]' if JOINT_NAMES_ORDERED[idx] == "DIP" else ''}")

S = CONFIG.S

def solve_tendon_tensions_preload_qp(
        S,
        tau,
        preload=10.0,
        max_tension=np.inf,
        tendon_weights=None,
        pair_weights=None,
        x0=None,
):
    """
    Solve tendon tensions by minimizing deviation from a common preload.

    Minimize:
        0.5 * sum_i w_i * (T_i - preload)^2
        + 0.5 * sum_(i,j,k) pair_w * (T_i - T_j)^2

    Subject to:
        S @ T = tau
        0 <= T_i <= max_tension

    Inputs
    ------
    S : (m,n) array
    tau : (m,) array
    preload : desired baseline tendon tension
    max_tension : scalar or length-n array
    tendon_weights : optional length-n weights
    pair_weights : optional list of tuples (i, j, w)
        penalizes difference between tendon i and j
    x0 : optional initial guess

    Returns
    -------
    T : solution tensions
    res : scipy optimize result
    """
    S = np.asarray(S, dtype=float)
    tau = np.asarray(tau, dtype=float)
    m, n = S.shape

    if tendon_weights is None:
        tendon_weights = np.ones(n, dtype=float)
    else:
        tendon_weights = np.asarray(tendon_weights, dtype=float)

    if np.isscalar(max_tension):
        ub = np.full(n, float(max_tension))
    else:
        ub = np.asarray(max_tension, dtype=float)

    lb = np.zeros(n, dtype=float)

    if x0 is None:
        x0 = np.full(n, preload, dtype=float)

        # project initial guess toward feasibility with least-squares correction
        try:
            dx, *_ = np.linalg.lstsq(S, tau - S @ x0, rcond=None)
            x0 = x0 + dx
        except np.linalg.LinAlgError:
            pass

        x0 = np.clip(x0, lb, ub)

    def obj(T):
        T = np.asarray(T, dtype=float)

        val = 0.5 * np.sum(tendon_weights * (T - preload)**2)

        if pair_weights is not None:
            for i, j, w in pair_weights:
                val += 0.5 * w * (T[i] - T[j])**2

        return val

    def grad(T):
        T = np.asarray(T, dtype=float)

        g = tendon_weights * (T - preload)

        if pair_weights is not None:
            for i, j, w in pair_weights:
                d = w * (T[i] - T[j])
                g[i] += d
                g[j] -= d

        return g

    cons = [{
        "type": "eq",
        "fun": lambda T: S @ T - tau,
        "jac": lambda T: S,
    }]

    bounds = [(lb[i], ub[i]) for i in range(n)]

    res = minimize(
        fun=obj,
        x0=x0,
        jac=grad,
        bounds=bounds,
        constraints=cons,
        method="SLSQP",
        options={"ftol": 1e-10, "maxiter": 1000},
    )

    return res.x, res

def solve_tendon_tensions(S, tau, optimization_option=None):
    if optimization_option is None:
        Ts, rnorm = nnls(S, tau)
        return Ts, rnorm
    m, n = S.shape
    c = np.zeros(n+1)
    c[-1] = -1.0
    A_eq = np.zeros((m, n+1))
    A_eq[:, :n] = S
    b_eq = tau

    A_ub = np.zeros((n, n+1))
    b_ub = np.zeros(n)
    results = []
    for i in range(n):
        A_ub[i, i] = -1.0
        A_ub[i, -1] = 1.0

    if not TEST_MOTOR_OPTIONS_VERBOSE:
        idx = int(len(MOTOR_OPTIONS_TENSIONS)*0.5)
        max_tension = MOTOR_OPTIONS_TENSIONS[idx][1]
        name = MOTOR_OPTIONS_TENSIONS[idx][0]
        bounds = [(0.0, max_tension)]*n + [(None, None)]
        res = linprog(c, A_ub, b_ub, A_eq, b_eq, bounds, method="highs")
        if res.success:
            msg = f"\nMotor: {name} with Max Tension: {max_tension:.2f} N.\n    Success!"
            results.append((msg, res))
            return results
        else:
            print(f"\nMotor: {name} with Max Tension: {max_tension:.2f}\n      Failed: {res.message}")
            return
    
    else:
        for motor_choice in MOTOR_OPTIONS_TENSIONS:
            bounds = [(0.0, motor_choice[1])]*n + [(None, None)]
            res = linprog(c, A_ub, b_ub, A_eq, b_eq, bounds, method="highs")
            if res.success:
                msg = f"\nMotor: {motor_choice[0]} with Max Tension: {motor_choice[1]}\n    Success!"
                results.append((msg, res))
            else:
                print(f"\nMotor: {motor_choice[0]} with Max Tension: {motor_choice[1]}\n      Failed: {res.message}")
        #print("\nFailed: Maximum torque exceeded for all motor choices for this S, joint_torque.")
        return results

# T1, rnorm1 = solve_tendon_tensions(S, tau_joint, None)
T1, rnorm1 = solve_tendon_tensions_preload_qp(S, tau_joint)
res = solve_tendon_tensions(S, tau_joint, "maximum minimum tensions")

print(f"\nTendon Tensions (N):")
for idx, tension in enumerate(T1):
    print(f"    {TENDON_NAMES_ORDERED[idx]}: {tension:.2f}")
print(f"\nresidual norm: {rnorm1}")
print(f"tau_hat: {S@T1}")

if True:
    for msg, re in res:
        print(msg)
        T2 = re.x[:-1]
        tstar = re.x[-1]
        print(f"\nTendon Tensions (N):\n    'Optimized for maximum minimum-tendon-tension.'\n")
        for idx, tension in enumerate(T2):
            print(f"    {TENDON_NAMES_ORDERED[idx]}: {tension:.2f}")
        print(f"\nt*: {tstar}")
        print(f"tau_hat: {S@T2}")

# ============================================ TENDON TENSIONS ===================================================

SHAFT_LENGTH_DEFAULT = 30
TERMINATION_OFFSET = 10

