import numpy as np

from rds_finger.model import FingerModel
from rds_finger.analysis.kinematics.tip_jacobian import joint_torques_from_tip_force
from rds_finger.analysis.kinematics.jacobian import moment_arm_matrix
from rds_finger.loads.loads import compute_all_pulley_and_shaft_loads
from rds_finger.loads.bearing_loads import compute_all_bearing_loads
from rds_finger.analysis.loads.feasibility import feasibility_report
from rds_finger.analysis.solvers.tensions import solve_tendon_tensions
from rds_finger.analysis.loads.motors import compute_drum_torques
from rds_finger.analysis.routing.spool_checks import check_spool_directions
from rds_finger.analysis.loads.bearing_life import compute_bearing_lives, worst_bearing_life
from rds_finger.analysis.loads.shaft_stress import compute_shaft_stresses, worst_shaft_stress

def analyze_tip_force(
    model: FingerModel,
    q: np.ndarray,
    F_tip_xyz: np.ndarray,
    preload: float = 0.0,
) -> dict:
    q = np.asarray(q, float).reshape(-1)
    F_tip_xyz = np.asarray(F_tip_xyz, float).reshape(3)

    tau = joint_torques_from_tip_force(model, q, F_tip_xyz)

    A = moment_arm_matrix(model, q)  # (n_dof, n_tendon)
    # T = nnls(A, tau)
    row_weights = np.array([0.1, 1.0, 1.0], float) 
    T = solve_tendon_tensions(
        model, q, tau,
        preload=preload,
        row_weights=row_weights,
        reg_w=1e-3,
    )

    if preload > 0.0:
        T = np.maximum(T, float(preload))

    tau_hat = A @ T
    err = float(np.linalg.norm(tau_hat - tau))
    feas = feasibility_report(A, tau, tau_hat)

    pulley_loads, shaft_loads = compute_all_pulley_and_shaft_loads(model, q, T)
    bearing_loads = compute_all_bearing_loads(model, q, T)
    bearing_lives = compute_bearing_lives(model, bearing_loads)
    shaft_stresses = compute_shaft_stresses(model, shaft_loads)

    worst_life = worst_bearing_life(bearing_lives)
    worst_stress = worst_shaft_stress(shaft_stresses)
    drum_torques = compute_drum_torques(model, T)
    spool_checks = check_spool_directions(model)

    return {
        "q": q,
        "F_tip_xyz": F_tip_xyz,
        "tau": tau,
        "A": A,
        "tensions": T,
        "tau_hat": tau_hat,
        "nnls_err": err,
        "pulley_loads": pulley_loads,
        "shaft_loads": shaft_loads,
        "bearing_loads": bearing_loads,
        "feasibility": feas,
        "drum_torques": drum_torques,
        "spool_checks": spool_checks,
        "bearing_lives": bearing_lives,
        "shaft_stresses": shaft_stresses,
        "worst_bearing_life": worst_life,
        "worst_shaft_stress": worst_stress,
    }