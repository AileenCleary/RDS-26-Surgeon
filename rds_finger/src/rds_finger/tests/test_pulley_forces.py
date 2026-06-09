import numpy as np

from rds_finger.statics.shaft_forces import build_shaft_loads_from_tendon_tensions
from rds_finger.config import TENDON_PATH
from rds_finger.statics.format_helpers import fmt_vec, print_pulley_by_pulley_forces
from rds_finger.config import tendon_tensions

if __name__ == "__main__":

    shaft_loads = build_shaft_loads_from_tendon_tensions(
        tendon_tensions=tendon_tensions,
        tendon_paths=TENDON_PATH,
    )

    print_pulley_by_pulley_forces(shaft_loads)
