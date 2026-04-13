from rds_finger.statics.bearing_forces import solve_and_check_all_bearings
from rds_finger.statics.format_helpers import print_bearing_results
from rds_finger.config import TENDON_PATH, SHAFT_BEARINGS
from rds_finger.config import tendon_tensions

if __name__ == "__main__":

    results = solve_and_check_all_bearings(
        tendon_tensions=tendon_tensions,
        tendon_paths=TENDON_PATH,
        shaft_bearings=SHAFT_BEARINGS,
        axial_side="left",
        rpm=60.0,
    )

    print_bearing_results(results)
