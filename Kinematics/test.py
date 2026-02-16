import config as cfg
from kinematics import RoboticFingerKinematics
from transmission import TendonTransmission
from shaft_analysis import compute_tendon_pulley_loads
from select_bearings import (
    Scenario,
    loads_by_shaft,
    compute_reactions_for_shaft,
    evaluate_bearing_for_scenario,
)

def analyze_tip_force(F_xy, q, mu=0.05, friction_mode="decay", use_tan_wrap=True, use_tan_force=True):
    kin = RoboticFingerKinematics(cfg.LINK_LENGTHS, cfg.FINGERTIP_OFFSET, cfg.COUPLING_RATIO)
    tau = kin.joint_torque_from_tip_force_xy(q, F_xy)
    trans = TendonTransmission(cfg.ALL_PULLEYS, cfg.TEN)