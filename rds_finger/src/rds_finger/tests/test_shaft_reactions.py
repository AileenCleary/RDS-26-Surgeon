"""
Test script: solve and print shaft bearing reactions from tendon tensions.

Adjust the CONFIG IMPORTS section if your project stores TENDON_PATH or shaft_bearings
in a different module.
"""

import numpy as np

from rds_finger.statics.bearing_forces import solve_all_bearing_reactions_from_tendon_tensions
from rds_finger.statics.format_helpers import fmt_vec, print_shaft_reactions
from rds_finger.config import TENDON_PATH, SHAFT_BEARINGS
from rds_finger.config import tendon_tensions

if __name__ == "__main__":

    reactions = solve_all_bearing_reactions_from_tendon_tensions(
        tendon_tensions=tendon_tensions,
        tendon_paths=TENDON_PATH,
        shaft_bearings=SHAFT_BEARINGS,
        axial_side="left",
    )

    print_shaft_reactions(reactions)
