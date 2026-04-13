import numpy as np

def fmt_vec(v, prec=2):
    v = np.asarray(v, dtype=float)
    return f"[{v[0]:8.{prec}f}, {v[1]:8.{prec}f}, {v[2]:8.{prec}f}]"

def print_shaft_reactions(reactions):
    print("\n" + "="*60)
    print("SHAFT REACTIONS")
    print("="*60)

    for shaft, r in reactions.items():
        print(f"\n--- Shaft {shaft} ---")

        print("Left Reaction:")
        print("  Total:  ", fmt_vec(r["left_reaction"]))
        print("  Radial: ", fmt_vec(r["left_radial"]),
              f" |mag|={r['left_radial_mag']:.2f}")
        print("  Axial:  ", fmt_vec(r["left_axial"]),
              f" |mag|={r['left_axial_mag']:.2f}")

        print("Right Reaction:")
        print("  Total:  ", fmt_vec(r["right_reaction"]))
        print("  Radial: ", fmt_vec(r["right_radial"]),
              f" |mag|={r['right_radial_mag']:.2f}")
        print("  Axial:  ", fmt_vec(r["right_axial"]),
              f" |mag|={r['right_axial_mag']:.2f}")

def print_bearing_results(results):
    print("\n" + "="*60)
    print("BEARING RESULTS")
    print("="*60)

    for shaft, data in results.items():
        print(f"\n--- Shaft {shaft} ---")

        for side in ["left_bearing_check", "right_bearing_check"]:
            b = data[side]
            label = "Left" if "left" in side else "Right"

            print(f"\n  {label} Bearing:")
            print(f"    Fr: {b['Fr']:.2f}")
            print(f"    Fa: {b['Fa']:.2f}")

            if "P0" in b:
                print(f"    P0: {b['P0']:.2f}")
                print(f"    Static SF: {b['static_sf']:.2f}")

            if "P" in b:
                print(f"    P: {b['P']:.2f}")
                print(f"    Dynamic Util: {b['dynamic_util']:.3f}")
                print(f"    L10 (rev, millions): {b['L10_rev_millions']:.2f}")

                if "L10_hours" in b:
                    print(f"    L10 (hours): {b['L10_hours']:.2f}")

def print_pulley_by_pulley_forces(shaft_loads: dict) -> None:
    print("\n================ PULLEY-BY-PULLEY FORCES ================\n")

    for shaft, loads in shaft_loads.items():
        print(f"{shaft}")

        if not loads:
            print("  (no pulley loads)\n")
            continue

        shaft_sum = np.zeros(3, dtype=float)

        for i, load in enumerate(loads, start=1):
            shaft_sum += load.f_net
            pulley_name = getattr(load.pulley, "name", f"pulley_{i}")

            print(f"  {i}. {pulley_name}")
            print(f"     center        = {fmt_vec(load.center)}")
            print(f"     axis          = {fmt_vec(load.axis)}")
            print(f"     v_in          = {fmt_vec(load.v_in)}")
            print(f"     v_out         = {fmt_vec(load.v_out)}")
            print(f"     t_in          = {load.t_in:.3f}")
            print(f"     t_out         = {load.t_out:.3f}")
            print(f"     f_in          = {fmt_vec(load.f_in)}")
            print(f"     f_out         = {fmt_vec(load.f_out)}")
            print(f"     f_net         = {fmt_vec(load.f_net)}")
            print(f"     |f_in|        = {np.linalg.norm(load.f_in):.3f}")
            print(f"     |f_out|       = {np.linalg.norm(load.f_out):.3f}")
            print(f"     |f_net|       = {np.linalg.norm(load.f_net):.3f}")
            print()

        print(f"  shaft_sum_f_net  = {fmt_vec(shaft_sum)}")
        print(f"  |shaft_sum_f_net|= {np.linalg.norm(shaft_sum):.3f}")
        print("\n------------------------------------------------\n")
