# # # import numpy as np
# # # import matplotlib.pyplot as plt

# # # from rds_finger.more.tangencies import (
# # #     Point3D,
# # #     Pulley,
# # #     plane_basis_with_vertical,
# # #     project_to_plane_coords,
# # #     choose_point_pulley_solution,
# # #     choose_pulley_pulley_solution,
# # #     tangents_point_pulley,
# # #     tangents_pulley_pulley,
# # # )


# # # def _circle_xy(center_2d, radius, n=400):
# # #     th = np.linspace(0, 2 * np.pi, n)
# # #     return np.column_stack([
# # #         center_2d[0] + radius * np.cos(th),
# # #         center_2d[1] + radius * np.sin(th),
# # #     ])


# # # def _set_equal_limits(ax, pts, pad_frac=0.2):
# # #     pts = np.asarray(pts, float)
# # #     mn = pts.min(axis=0)
# # #     mx = pts.max(axis=0)
# # #     pad = pad_frac * max(mx[0] - mn[0], mx[1] - mn[1], 1.0)
# # #     ax.set_xlim(mn[0] - pad, mx[0] + pad)
# # #     ax.set_ylim(mn[1] - pad, mx[1] + pad)
# # #     ax.set_aspect("equal")
# # #     ax.grid(True)


# # # def _base_geometry(point1, point2, pulley1, pulley2):
# # #     ex, ey, n = plane_basis_with_vertical(
# # #         pulley1.axis, vertical_hint=np.array([0.0, 0.0, 1.0])
# # #     )
# # #     origin = pulley1.center

# # #     p1_2d = project_to_plane_coords(point1.center, origin, ex, ey)
# # #     p2_2d = project_to_plane_coords(point2.center, origin, ex, ey)
# # #     c1_2d = project_to_plane_coords(pulley1.center, origin, ex, ey)
# # #     c2_2d = project_to_plane_coords(pulley2.center, origin, ex, ey)

# # #     circ1 = _circle_xy(c1_2d, pulley1.radius)
# # #     circ2 = _circle_xy(c2_2d, pulley2.radius)

# # #     return {
# # #         "ex": ex,
# # #         "ey": ey,
# # #         "n": n,
# # #         "origin": origin,
# # #         "p1_2d": p1_2d,
# # #         "p2_2d": p2_2d,
# # #         "c1_2d": c1_2d,
# # #         "c2_2d": c2_2d,
# # #         "circ1": circ1,
# # #         "circ2": circ2,
# # #     }


# # # def save_segment1_options(point1, pulley1, geom, filename):
# # #     sols = tangents_point_pulley(point1, pulley1)
# # #     chosen = choose_point_pulley_solution(point1, pulley1)

# # #     p1_2d = geom["p1_2d"]
# # #     c1_2d = geom["c1_2d"]
# # #     circ1 = geom["circ1"]
# # #     ey = geom["ey"]

# # #     nsol = len(sols)
# # #     fig, axes = plt.subplots(1, nsol, figsize=(5 * nsol, 5))
# # #     if nsol == 1:
# # #         axes = [axes]

# # #     all_pts = [p1_2d, c1_2d]
# # #     all_pts.extend([s["tangent_point_2d"] for s in sols])
# # #     all_pts.extend(circ1)

# # #     for i, (ax, sol) in enumerate(zip(axes, sols), start=1):
# # #         t = sol["tangent_point_2d"]
# # #         is_chosen = np.allclose(t, chosen["tangent_point_2d"])

# # #         ax.plot(circ1[:, 0], circ1[:, 1], label=pulley1.name)
# # #         ax.plot([p1_2d[0], t[0]], [p1_2d[1], t[1]], linewidth=2.0)
# # #         ax.plot([c1_2d[0], t[0]], [c1_2d[1], t[1]], "--", linewidth=1.0)
# # #         ax.scatter([p1_2d[0]], [p1_2d[1]], marker="o", s=60)
# # #         ax.scatter([c1_2d[0]], [c1_2d[1]], marker="x", s=80)
# # #         ax.scatter([t[0]], [t[1]], marker="s", s=60)

# # #         ax.text(p1_2d[0], p1_2d[1], "  Point1")
# # #         ax.text(c1_2d[0], c1_2d[1], "  Pulley1")

# # #         ax.set_title(
# # #             f"Seg1 option {i}\n"
# # #             f"{'CHOSEN' if is_chosen else 'not chosen'}\n"
# # #             f"Pulley1={pulley1.direction}"
# # #         )
# # #         ax.set_xlabel("horizontal")
# # #         ax.set_ylabel("vertical (+up)")
# # #         _set_equal_limits(ax, all_pts)

# # #     fig.suptitle(
# # #         "Point1 ↔ Pulley1 possible tangent solutions\n"
# # #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}",
# # #         y=1.02
# # #     )
# # #     fig.tight_layout()
# # #     fig.savefig(filename, dpi=220, bbox_inches="tight")
# # #     plt.close(fig)
# # #     print(f"Saved {filename}")


# # # def save_segment2_options(pulley1, pulley2, geom, filename):
# # #     sols = tangents_pulley_pulley(pulley1, pulley2)
# # #     chosen = choose_pulley_pulley_solution(pulley1, pulley2)

# # #     c1_2d = geom["c1_2d"]
# # #     c2_2d = geom["c2_2d"]
# # #     circ1 = geom["circ1"]
# # #     circ2 = geom["circ2"]
# # #     ey = geom["ey"]

# # #     nsol = len(sols)
# # #     fig, axes = plt.subplots(1, nsol, figsize=(5 * nsol, 5))
# # #     if nsol == 1:
# # #         axes = [axes]

# # #     all_pts = [c1_2d, c2_2d]
# # #     for s in sols:
# # #         all_pts.extend([s["t1_2d"], s["t2_2d"]])
# # #     all_pts.extend(circ1)
# # #     all_pts.extend(circ2)

# # #     for i, (ax, sol) in enumerate(zip(axes, sols), start=1):
# # #         t1 = sol["t1_2d"]
# # #         t2 = sol["t2_2d"]
# # #         d = sol["direction_2d"]

# # #         is_chosen = (
# # #             np.allclose(t1, chosen["t1_2d"]) and np.allclose(t2, chosen["t2_2d"])
# # #         )

# # #         L = 100.0
# # #         a = t1 - L * d
# # #         b = t1 + L * d

# # #         ax.plot(circ1[:, 0], circ1[:, 1], label=pulley1.name)
# # #         ax.plot(circ2[:, 0], circ2[:, 1], label=pulley2.name)
# # #         ax.plot([a[0], b[0]], [a[1], b[1]], linewidth=2.0)
# # #         ax.plot([c1_2d[0], t1[0]], [c1_2d[1], t1[1]], "--", linewidth=1.0)
# # #         ax.plot([c2_2d[0], t2[0]], [c2_2d[1], t2[1]], "--", linewidth=1.0)
# # #         ax.scatter([c1_2d[0], c2_2d[0]], [c1_2d[1], c2_2d[1]], marker="x", s=80)
# # #         ax.scatter([t1[0], t2[0]], [t1[1], t2[1]], marker="s", s=60)

# # #         ax.text(c1_2d[0], c1_2d[1], "  Pulley1")
# # #         ax.text(c2_2d[0], c2_2d[1], "  Pulley2")

# # #         ax.set_title(
# # #             f"Seg2 option {i}\n"
# # #             f"{sol['family']}\n"
# # #             f"{'CHOSEN' if is_chosen else 'not chosen'}"
# # #         )
# # #         ax.set_xlabel("horizontal")
# # #         ax.set_ylabel("vertical (+up)")
# # #         _set_equal_limits(ax, all_pts)

# # #     fig.suptitle(
# # #         "Pulley1 ↔ Pulley2 possible tangent solutions\n"
# # #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
# # #         f"Pulley1={pulley1.direction}, Pulley2={pulley2.direction}",
# # #         y=1.04
# # #     )
# # #     fig.tight_layout()
# # #     fig.savefig(filename, dpi=220, bbox_inches="tight")
# # #     plt.close(fig)
# # #     print(f"Saved {filename}")


# # # def save_segment3_options(point2, pulley2, geom, filename):
# # #     sols = tangents_point_pulley(point2, pulley2)
# # #     chosen = choose_point_pulley_solution(point2, pulley2)

# # #     p2_2d = geom["p2_2d"]
# # #     c2_2d = geom["c2_2d"]
# # #     circ2 = geom["circ2"]
# # #     ey = geom["ey"]

# # #     nsol = len(sols)
# # #     fig, axes = plt.subplots(1, nsol, figsize=(5 * nsol, 5))
# # #     if nsol == 1:
# # #         axes = [axes]

# # #     all_pts = [p2_2d, c2_2d]
# # #     all_pts.extend([s["tangent_point_2d"] for s in sols])
# # #     all_pts.extend(circ2)

# # #     for i, (ax, sol) in enumerate(zip(axes, sols), start=1):
# # #         t = sol["tangent_point_2d"]
# # #         is_chosen = np.allclose(t, chosen["tangent_point_2d"])

# # #         ax.plot(circ2[:, 0], circ2[:, 1], label=pulley2.name)
# # #         ax.plot([t[0], p2_2d[0]], [t[1], p2_2d[1]], linewidth=2.0)
# # #         ax.plot([c2_2d[0], t[0]], [c2_2d[1], t[1]], "--", linewidth=1.0)
# # #         ax.scatter([p2_2d[0]], [p2_2d[1]], marker="o", s=60)
# # #         ax.scatter([c2_2d[0]], [c2_2d[1]], marker="x", s=80)
# # #         ax.scatter([t[0]], [t[1]], marker="s", s=60)

# # #         ax.text(p2_2d[0], p2_2d[1], "  Point2")
# # #         ax.text(c2_2d[0], c2_2d[1], "  Pulley2")

# # #         ax.set_title(
# # #             f"Seg3 option {i}\n"
# # #             f"{'CHOSEN' if is_chosen else 'not chosen'}\n"
# # #             f"Pulley2={pulley2.direction}"
# # #         )
# # #         ax.set_xlabel("horizontal")
# # #         ax.set_ylabel("vertical (+up)")
# # #         _set_equal_limits(ax, all_pts)

# # #     fig.suptitle(
# # #         "Pulley2 ↔ Point2 possible tangent solutions\n"
# # #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}",
# # #         y=1.02
# # #     )
# # #     fig.tight_layout()
# # #     fig.savefig(filename, dpi=220, bbox_inches="tight")
# # #     plt.close(fig)
# # #     print(f"Saved {filename}")


# # # def save_all_route_combinations(point1, point2, pulley1, pulley2, geom, filename):
# # #     seg1_sols = tangents_point_pulley(point1, pulley1)
# # #     seg2_sols = tangents_pulley_pulley(pulley1, pulley2)
# # #     seg3_sols = tangents_point_pulley(point2, pulley2)

# # #     chosen1 = choose_point_pulley_solution(point1, pulley1)
# # #     chosen2 = choose_pulley_pulley_solution(pulley1, pulley2)
# # #     chosen3 = choose_point_pulley_solution(point2, pulley2)

# # #     p1_2d = geom["p1_2d"]
# # #     p2_2d = geom["p2_2d"]
# # #     c1_2d = geom["c1_2d"]
# # #     c2_2d = geom["c2_2d"]
# # #     circ1 = geom["circ1"]
# # #     circ2 = geom["circ2"]
# # #     ey = geom["ey"]

# # #     combos = []
# # #     for i, s1 in enumerate(seg1_sols):
# # #         for j, s2 in enumerate(seg2_sols):
# # #             for k, s3 in enumerate(seg3_sols):
# # #                 combos.append((i, j, k, s1, s2, s3))

# # #     n = len(combos)
# # #     ncols = 4
# # #     nrows = int(np.ceil(n / ncols))
# # #     fig, axes = plt.subplots(nrows, ncols, figsize=(4.5 * ncols, 4.5 * nrows))
# # #     axes = np.atleast_2d(axes)

# # #     all_pts = [p1_2d, p2_2d, c1_2d, c2_2d]
# # #     all_pts.extend(circ1)
# # #     all_pts.extend(circ2)
# # #     for _, _, _, s1, s2, s3 in combos:
# # #         all_pts.extend([
# # #             s1["tangent_point_2d"],
# # #             s2["t1_2d"],
# # #             s2["t2_2d"],
# # #             s3["tangent_point_2d"],
# # #         ])

# # #     for ax in axes.flat[n:]:
# # #         ax.axis("off")

# # #     for idx, (i, j, k, s1, s2, s3) in enumerate(combos):
# # #         ax = axes.flat[idx]

# # #         t_p1_pul1 = s1["tangent_point_2d"]
# # #         t_pul1_pul2_1 = s2["t1_2d"]
# # #         t_pul1_pul2_2 = s2["t2_2d"]
# # #         t_pul2_p2 = s3["tangent_point_2d"]

# # #         is_chosen = (
# # #             np.allclose(t_p1_pul1, chosen1["tangent_point_2d"]) and
# # #             np.allclose(t_pul1_pul2_1, chosen2["t1_2d"]) and
# # #             np.allclose(t_pul1_pul2_2, chosen2["t2_2d"]) and
# # #             np.allclose(t_pul2_p2, chosen3["tangent_point_2d"])
# # #         )

# # #         ax.plot(circ1[:, 0], circ1[:, 1])
# # #         ax.plot(circ2[:, 0], circ2[:, 1])

# # #         ax.plot([p1_2d[0], t_p1_pul1[0]], [p1_2d[1], t_p1_pul1[1]], linewidth=2.0)
# # #         ax.plot(
# # #             [t_pul1_pul2_1[0], t_pul1_pul2_2[0]],
# # #             [t_pul1_pul2_1[1], t_pul1_pul2_2[1]],
# # #             linewidth=2.0,
# # #         )
# # #         ax.plot([t_pul2_p2[0], p2_2d[0]], [t_pul2_p2[1], p2_2d[1]], linewidth=2.0)

# # #         ax.plot([c1_2d[0], t_p1_pul1[0]], [c1_2d[1], t_p1_pul1[1]], "--", linewidth=1.0)
# # #         ax.plot([c1_2d[0], t_pul1_pul2_1[0]], [c1_2d[1], t_pul1_pul2_1[1]], "--", linewidth=1.0)
# # #         ax.plot([c2_2d[0], t_pul1_pul2_2[0]], [c2_2d[1], t_pul1_pul2_2[1]], "--", linewidth=1.0)
# # #         ax.plot([c2_2d[0], t_pul2_p2[0]], [c2_2d[1], t_pul2_p2[1]], "--", linewidth=1.0)

# # #         ax.scatter([p1_2d[0], p2_2d[0]], [p1_2d[1], p2_2d[1]], marker="o", s=40)
# # #         ax.scatter([c1_2d[0], c2_2d[0]], [c1_2d[1], c2_2d[1]], marker="x", s=50)

# # #         tangent_pts = np.array([
# # #             t_p1_pul1,
# # #             t_pul1_pul2_1,
# # #             t_pul1_pul2_2,
# # #             t_pul2_p2,
# # #         ])
# # #         ax.scatter(tangent_pts[:, 0], tangent_pts[:, 1], marker="s", s=35)

# # #         ax.set_title(
# # #             f"({i+1},{j+1},{k+1})"
# # #             + ("\nCHOSEN" if is_chosen else "")
# # #         )
# # #         ax.set_xlabel("horizontal")
# # #         ax.set_ylabel("vertical (+up)")
# # #         _set_equal_limits(ax, all_pts)

# # #     fig.suptitle(
# # #         "All route combinations: (seg1 option, seg2 option, seg3 option)\n"
# # #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
# # #         "Note: these are not globally filtered for collisions/intersections yet.",
# # #         y=1.02
# # #     )
# # #     fig.tight_layout()
# # #     fig.savefig(filename, dpi=220, bbox_inches="tight")
# # #     plt.close(fig)
# # #     print(f"Saved {filename}")


# # # def save_two_pulley_system_example(base="example_two_pulley_system"):
# # #     point1 = Point3D(center=[-4.0, 0.0, -1.0], name="Point1")
# # #     point2 = Point3D(center=[10.0, 0.0, 3.5], name="Point2")

# # #     pulley1 = Pulley(
# # #         center=[0.0, 0.0, 0.0],
# # #         radius=1.5,
# # #         axis=[0.0, 1.0, 0.0],
# # #         direction="CW",
# # #         name="Pulley1",
# # #     )

# # #     pulley2 = Pulley(
# # #         center=[5.5, 0.0, 1.5],
# # #         radius=1.0,
# # #         axis=[0.0, 1.0, 0.0],
# # #         direction="CCW",
# # #         name="Pulley2",
# # #     )

# # #     geom = _base_geometry(point1, point2, pulley1, pulley2)

# # #     # chosen segments
# # #     seg1 = choose_point_pulley_solution(point1, pulley1)
# # #     seg2 = choose_pulley_pulley_solution(pulley1, pulley2)
# # #     seg3 = choose_point_pulley_solution(point2, pulley2)

# # #     p1_2d = geom["p1_2d"]
# # #     p2_2d = geom["p2_2d"]
# # #     c1_2d = geom["c1_2d"]
# # #     c2_2d = geom["c2_2d"]
# # #     circ1 = geom["circ1"]
# # #     circ2 = geom["circ2"]
# # #     ey = geom["ey"]

# # #     t_p1_pul1 = seg1["tangent_point_2d"]
# # #     t_pul1_pul2_1 = seg2["t1_2d"]
# # #     t_pul1_pul2_2 = seg2["t2_2d"]
# # #     t_pul2_p2 = seg3["tangent_point_2d"]

# # #     # chosen path figure
# # #     fig, ax = plt.subplots(figsize=(10, 6))
# # #     ax.plot(circ1[:, 0], circ1[:, 1], label=pulley1.name)
# # #     ax.plot(circ2[:, 0], circ2[:, 1], label=pulley2.name)

# # #     ax.plot([p1_2d[0], t_p1_pul1[0]], [p1_2d[1], t_p1_pul1[1]], linewidth=2.0, label="Point1 -> Pulley1")
# # #     ax.plot([t_pul1_pul2_1[0], t_pul1_pul2_2[0]], [t_pul1_pul2_1[1], t_pul1_pul2_2[1]], linewidth=2.0, label="Pulley1 -> Pulley2")
# # #     ax.plot([t_pul2_p2[0], p2_2d[0]], [t_pul2_p2[1], p2_2d[1]], linewidth=2.0, label="Pulley2 -> Point2")

# # #     ax.plot([c1_2d[0], t_p1_pul1[0]], [c1_2d[1], t_p1_pul1[1]], "--", linewidth=1.0)
# # #     ax.plot([c1_2d[0], t_pul1_pul2_1[0]], [c1_2d[1], t_pul1_pul2_1[1]], "--", linewidth=1.0)
# # #     ax.plot([c2_2d[0], t_pul1_pul2_2[0]], [c2_2d[1], t_pul1_pul2_2[1]], "--", linewidth=1.0)
# # #     ax.plot([c2_2d[0], t_pul2_p2[0]], [c2_2d[1], t_pul2_p2[1]], "--", linewidth=1.0)

# # #     ax.scatter([p1_2d[0], p2_2d[0]], [p1_2d[1], p2_2d[1]], marker="o", s=60)
# # #     ax.scatter([c1_2d[0], c2_2d[0]], [c1_2d[1], c2_2d[1]], marker="x", s=80)

# # #     tangent_pts = np.array([
# # #         t_p1_pul1,
# # #         t_pul1_pul2_1,
# # #         t_pul1_pul2_2,
# # #         t_pul2_p2,
# # #     ])
# # #     ax.scatter(tangent_pts[:, 0], tangent_pts[:, 1], marker="s", s=50)

# # #     ax.text(p1_2d[0], p1_2d[1], "  Point1")
# # #     ax.text(p2_2d[0], p2_2d[1], "  Point2")
# # #     ax.text(c1_2d[0], c1_2d[1], "  Pulley1")
# # #     ax.text(c2_2d[0], c2_2d[1], "  Pulley2")

# # #     ax.set_title(
# # #         "Chosen tangent path for Point1 <-> Pulley1 <-> Pulley2 <-> Point2\n"
# # #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
# # #         f"Pulley1={pulley1.direction}, Pulley2={pulley2.direction}"
# # #     )
# # #     ax.set_xlabel("horizontal")
# # #     ax.set_ylabel("vertical (+up)")
# # #     ax.legend()
# # #     _set_equal_limits(ax, np.vstack([circ1, circ2, p1_2d, p2_2d, tangent_pts]))

# # #     fig.tight_layout()
# # #     chosen_name = f"{base}_chosen.png"
# # #     fig.savefig(chosen_name, dpi=220, bbox_inches="tight")
# # #     plt.close(fig)
# # #     print(f"Saved {chosen_name}")

# # #     print("Chosen tangent points:")
# # #     print("  Point1 -> Pulley1:", t_p1_pul1)
# # #     print("  Pulley1 -> Pulley2 on Pulley1:", t_pul1_pul2_1)
# # #     print("  Pulley1 -> Pulley2 on Pulley2:", t_pul1_pul2_2)
# # #     print("  Pulley2 -> Point2:", t_pul2_p2)

# # #     # additional saved figures
# # #     save_segment1_options(point1, pulley1, geom, f"{base}_seg1_options.png")
# # #     save_segment2_options(pulley1, pulley2, geom, f"{base}_seg2_options.png")
# # #     save_segment3_options(point2, pulley2, geom, f"{base}_seg3_options.png")
# # #     save_all_route_combinations(point1, point2, pulley1, pulley2, geom, f"{base}_all_combos.png")


# # # if __name__ == "__main__":
# # #     save_two_pulley_system_example()

# # import numpy as np
# # import matplotlib.pyplot as plt

# # from rds_finger.more.tangencies import (
# #     Point3D,
# #     Pulley,
# #     plane_basis_with_vertical,
# #     project_to_plane_coords,
# #     tangents_point_pulley,
# #     choose_point_pulley_solution,
# # )


# # def verify_tangent_point_on_pulley(tangent_point_2d, center_2d, radius, atol=1e-9):
# #     return np.isclose(np.linalg.norm(tangent_point_2d - center_2d), radius, atol=atol)


# # def verify_tangent_condition(point_2d, center_2d, tangent_point_2d, atol=1e-9):
# #     radius_vec = tangent_point_2d - center_2d
# #     line_vec = point_2d - tangent_point_2d
# #     return np.isclose(np.dot(radius_vec, line_vec), 0.0, atol=atol)


# # def circle_xy(center_2d, radius, n=400):
# #     th = np.linspace(0, 2 * np.pi, n)
# #     return np.column_stack([
# #         center_2d[0] + radius * np.cos(th),
# #         center_2d[1] + radius * np.sin(th),
# #     ])


# # def set_equal_limits(ax, pts, pad_frac=0.2):
# #     pts = np.asarray(pts, float)
# #     mn = pts.min(axis=0)
# #     mx = pts.max(axis=0)
# #     pad = pad_frac * max(mx[0] - mn[0], mx[1] - mn[1], 1.0)
# #     ax.set_xlim(mn[0] - pad, mx[0] + pad)
# #     ax.set_ylim(mn[1] - pad, mx[1] + pad)
# #     ax.set_aspect("equal")
# #     ax.grid(True)


# # def save_debug_point2_pulley2(base="debug_point2_pulley2"):
# #     # Same geometry as your two-pulley example
# #     point2 = Point3D(center=[10.0, 0.0, 3.5], name="Point2")

# #     pulley2 = Pulley(
# #         center=[5.5, 0.0, 1.5],
# #         radius=1.0,
# #         axis=[0.0, 1.0, 0.0],
# #         direction="CCW",
# #         name="Pulley2",
# #     )

# #     # Build the plotting plane directly from pulley2
# #     ex, ey, n = plane_basis_with_vertical(
# #         pulley2.axis,
# #         vertical_hint=np.array([0.0, 0.0, 1.0]),
# #     )
# #     origin = pulley2.center

# #     # Project point and pulley center into 2D
# #     p2_2d = project_to_plane_coords(point2.center, origin, ex, ey)
# #     c2_2d = project_to_plane_coords(pulley2.center, origin, ex, ey)

# #     # Solve all point-pulley tangents and chosen tangent
# #     sols = tangents_point_pulley(point2, pulley2)
# #     chosen = choose_point_pulley_solution(point2, pulley2)

# #     circ2 = circle_xy(c2_2d, pulley2.radius)

# #     print("==== Debug: Point2 <-> Pulley2 ====")
# #     print("Projected point2:", p2_2d)
# #     print("Projected pulley2 center:", c2_2d)
# #     print("Selection vertical (+up):", ey)
# #     print("Pulley2 direction:", pulley2.direction)
# #     print()

# #     for i, sol in enumerate(sols, start=1):
# #         t = sol["tangent_point_2d"]
# #         on_circle = verify_tangent_point_on_pulley(t, c2_2d, pulley2.radius)
# #         tangent_ok = verify_tangent_condition(p2_2d, c2_2d, t)

# #         print(f"Solution {i}")
# #         print("  tangent point 2D:", t)
# #         print("  tangent point 3D:", sol["tangent_point_3d"])
# #         print("  distance to center:", np.linalg.norm(t - c2_2d))
# #         print("  expected radius   :", pulley2.radius)
# #         print("  on circle?        :", on_circle)
# #         print("  tangent condition :", tangent_ok)
# #         print("  signed vertical   :", t[1] - c2_2d[1])
# #         print()

# #     print("Chosen tangent point 2D:", chosen["tangent_point_2d"])
# #     print("Chosen tangent point 3D:", chosen["tangent_point_3d"])
# #     print()

# #     # ------------------------------------------------------------
# #     # Figure 1: all solutions
# #     # ------------------------------------------------------------
# #     fig, axes = plt.subplots(1, len(sols), figsize=(5 * len(sols), 5))
# #     if len(sols) == 1:
# #         axes = [axes]

# #     all_pts = [p2_2d, c2_2d]
# #     all_pts.extend(circ2)
# #     all_pts.extend([s["tangent_point_2d"] for s in sols])

# #     for i, (ax, sol) in enumerate(zip(axes, sols), start=1):
# #         t = sol["tangent_point_2d"]
# #         on_circle = verify_tangent_point_on_pulley(t, c2_2d, pulley2.radius)
# #         tangent_ok = verify_tangent_condition(p2_2d, c2_2d, t)
# #         is_chosen = np.allclose(t, chosen["tangent_point_2d"])

# #         ax.plot(circ2[:, 0], circ2[:, 1], label="Pulley2")
# #         ax.plot([t[0], p2_2d[0]], [t[1], p2_2d[1]], linewidth=2.0, label="tangent line")
# #         ax.plot([c2_2d[0], t[0]], [c2_2d[1], t[1]], "--", linewidth=1.0, label="radius")
# #         ax.scatter([p2_2d[0]], [p2_2d[1]], marker="o", s=60)
# #         ax.scatter([c2_2d[0]], [c2_2d[1]], marker="x", s=80)
# #         ax.scatter([t[0]], [t[1]], marker="s", s=60)

# #         ax.text(p2_2d[0], p2_2d[1], "  Point2")
# #         ax.text(c2_2d[0], c2_2d[1], "  Pulley2")

# #         ax.set_title(
# #             f"Solution {i}"
# #             f"{'  (CHOSEN)' if is_chosen else ''}\n"
# #             f"on_circle={on_circle}, tangent={tangent_ok}"
# #         )
# #         ax.set_xlabel("horizontal")
# #         ax.set_ylabel("vertical (+up)")
# #         set_equal_limits(ax, all_pts)

# #     fig.suptitle(
# #         "Point2 <-> Pulley2: all tangent solutions\n"
# #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
# #         f"Pulley2 direction = {pulley2.direction}",
# #         y=1.02,
# #     )
# #     fig.tight_layout()
# #     fig.savefig(f"{base}_all_solutions.png", dpi=220, bbox_inches="tight")
# #     plt.close(fig)

# #     # ------------------------------------------------------------
# #     # Figure 2: chosen vs unchosen together
# #     # ------------------------------------------------------------
# #     fig, ax = plt.subplots(figsize=(7, 6))

# #     ax.plot(circ2[:, 0], circ2[:, 1], label="Pulley2")
# #     ax.scatter([p2_2d[0]], [p2_2d[1]], marker="o", s=60)
# #     ax.scatter([c2_2d[0]], [c2_2d[1]], marker="x", s=80)
# #     ax.text(p2_2d[0], p2_2d[1], "  Point2")
# #     ax.text(c2_2d[0], c2_2d[1], "  Pulley2")

# #     for i, sol in enumerate(sols, start=1):
# #         t = sol["tangent_point_2d"]
# #         is_chosen = np.allclose(t, chosen["tangent_point_2d"])

# #         ax.plot(
# #             [t[0], p2_2d[0]],
# #             [t[1], p2_2d[1]],
# #             linewidth=2.5 if is_chosen else 1.5,
# #             alpha=1.0 if is_chosen else 0.6,
# #             label=f"solution {i}" + (" chosen" if is_chosen else ""),
# #         )
# #         ax.plot(
# #             [c2_2d[0], t[0]],
# #             [c2_2d[1], t[1]],
# #             "--",
# #             linewidth=1.0,
# #             alpha=1.0 if is_chosen else 0.6,
# #         )
# #         ax.scatter([t[0]], [t[1]], marker="s", s=70 if is_chosen else 45)

# #     ax.set_title(
# #         "Point2 <-> Pulley2: chosen and alternate solutions together\n"
# #         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
# #         "rule: CW=lower, CCW=upper"
# #     )
# #     ax.set_xlabel("horizontal")
# #     ax.set_ylabel("vertical (+up)")
# #     set_equal_limits(ax, all_pts)
# #     ax.legend()

# #     fig.tight_layout()
# #     fig.savefig(f"{base}_chosen_vs_alternates.png", dpi=220, bbox_inches="tight")
# #     plt.close(fig)

# #     print(f"Saved {base}_all_solutions.png")
# #     print(f"Saved {base}_chosen_vs_alternates.png")


# # if __name__ == "__main__":
# #     save_debug_point2_pulley2()

# import itertools
# import numpy as np
# import matplotlib.pyplot as plt

# from rds_finger.more.tangencies import (
#     Point3D,
#     Pulley,
#     plane_basis_with_vertical,
#     project_to_plane_coords,
#     tangents_point_pulley,
#     tangents_pulley_pulley,
# )


# # ============================================================
# # Basic helpers
# # ============================================================

# def circle_xy(center_2d, radius, n=400):
#     th = np.linspace(0.0, 2.0 * np.pi, n)
#     return np.column_stack([
#         center_2d[0] + radius * np.cos(th),
#         center_2d[1] + radius * np.sin(th),
#     ])


# def set_equal_limits(ax, pts, pad_frac=0.2):
#     pts = np.asarray(pts, float)
#     mn = pts.min(axis=0)
#     mx = pts.max(axis=0)
#     pad = pad_frac * max(mx[0] - mn[0], mx[1] - mn[1], 1.0)
#     ax.set_xlim(mn[0] - pad, mx[0] + pad)
#     ax.set_ylim(mn[1] - pad, mx[1] + pad)
#     ax.set_aspect("equal")
#     ax.grid(True)


# def preferred_sign_from_direction(direction):
#     # CW => lower => negative vertical
#     # CCW => upper => positive vertical
#     return -1.0 if direction == "CW" else 1.0


# def signed_height(pt, center):
#     return float(pt[1] - center[1])


# # ============================================================
# # Geometry / collision helpers
# # ============================================================

# def segment_distance_to_point(p0, p1, c, tol=1e-12):
#     """
#     Minimum distance from point c to finite segment p0->p1 in 2D.
#     """
#     p0 = np.asarray(p0, float)
#     p1 = np.asarray(p1, float)
#     c = np.asarray(c, float)

#     d = p1 - p0
#     a = np.dot(d, d)
#     if a < tol:
#         return np.linalg.norm(c - p0)

#     t = np.dot(c - p0, d) / a
#     t = np.clip(t, 0.0, 1.0)
#     q = p0 + t * d
#     return np.linalg.norm(c - q)


# def segment_hits_circle_interior(p0, p1, c, r, tol=1e-9):
#     """
#     True if the finite segment p0->p1 enters the circle interior.

#     Using < r - tol, not <= r, so exact tangency/contact is allowed.
#     """
#     return segment_distance_to_point(p0, p1, c) < (r - tol)


# # ============================================================
# # Route generation and filtering
# # ============================================================

# def build_common_geometry(point1, point2, pulley1, pulley2):
#     ex, ey, n = plane_basis_with_vertical(
#         pulley1.axis,
#         vertical_hint=np.array([0.0, 0.0, 1.0]),
#     )
#     origin = pulley1.center

#     p1_2d = project_to_plane_coords(point1.center, origin, ex, ey)
#     p2_2d = project_to_plane_coords(point2.center, origin, ex, ey)
#     c1_2d = project_to_plane_coords(pulley1.center, origin, ex, ey)
#     c2_2d = project_to_plane_coords(pulley2.center, origin, ex, ey)

#     return {
#         "ex": ex,
#         "ey": ey,
#         "n": n,
#         "origin": origin,
#         "p1_2d": p1_2d,
#         "p2_2d": p2_2d,
#         "c1_2d": c1_2d,
#         "c2_2d": c2_2d,
#         "circ1": circle_xy(c1_2d, pulley1.radius),
#         "circ2": circle_xy(c2_2d, pulley2.radius),
#     }


# def generate_route_candidates(point1, point2, pulley1, pulley2):
#     seg1_sols = tangents_point_pulley(point1, pulley1)
#     seg2_sols = tangents_pulley_pulley(pulley1, pulley2)
#     seg3_sols = tangents_point_pulley(point2, pulley2)

#     routes = []
#     idx = 0
#     for i, s1 in enumerate(seg1_sols):
#         for j, s2 in enumerate(seg2_sols):
#             for k, s3 in enumerate(seg3_sols):
#                 routes.append({
#                     "route_id": idx,
#                     "seg1_idx": i,
#                     "seg2_idx": j,
#                     "seg3_idx": k,
#                     "seg1": s1,
#                     "seg2": s2,
#                     "seg3": s3,
#                 })
#                 idx += 1
#     return routes


# def route_collision_report(route, geom, pulley1, pulley2):
#     """
#     Check the three straight segments against the wrong pulley interiors.

#     Route structure:
#       point1 -> t11
#       t21 -> t22
#       t32 -> point2

#     where:
#       t11 = seg1 tangency on pulley1
#       t21 = seg2 tangency on pulley1
#       t22 = seg2 tangency on pulley2
#       t32 = seg3 tangency on pulley2
#     """
#     p1 = geom["p1_2d"]
#     p2 = geom["p2_2d"]
#     c1 = geom["c1_2d"]
#     c2 = geom["c2_2d"]

#     t11 = route["seg1"]["tangent_point_2d"]
#     t21 = route["seg2"]["t1_2d"]
#     t22 = route["seg2"]["t2_2d"]
#     t32 = route["seg3"]["tangent_point_2d"]

#     report = {
#         "seg1_hits_pulley2": segment_hits_circle_interior(p1, t11, c2, pulley2.radius),
#         "seg2_hits_pulley1": segment_hits_circle_interior(t21, t22, c1, pulley1.radius),
#         "seg2_hits_pulley2": segment_hits_circle_interior(t21, t22, c2, pulley2.radius),
#         "seg3_hits_pulley1": segment_hits_circle_interior(t32, p2, c1, pulley1.radius),
#     }
#     report["valid"] = not any(report.values())
#     return report


# def score_route(route, geom, pulley1, pulley2):
#     """
#     Score by how well the tangency heights match pulley direction preferences.
#     Higher is better.

#     pulley1 uses:
#       seg1 tangency on pulley1
#       seg2 pulley1 tangency

#     pulley2 uses:
#       seg2 pulley2 tangency
#       seg3 tangency on pulley2
#     """
#     c1 = geom["c1_2d"]
#     c2 = geom["c2_2d"]

#     t11 = route["seg1"]["tangent_point_2d"]
#     t21 = route["seg2"]["t1_2d"]
#     t22 = route["seg2"]["t2_2d"]
#     t32 = route["seg3"]["tangent_point_2d"]

#     pref1 = preferred_sign_from_direction(pulley1.direction)
#     pref2 = preferred_sign_from_direction(pulley2.direction)

#     score = 0.0
#     score += pref1 * signed_height(t11, c1)
#     score += pref1 * signed_height(t21, c1)
#     score += pref2 * signed_height(t22, c2)
#     score += pref2 * signed_height(t32, c2)
#     return score


# def filter_and_score_routes(routes, geom, pulley1, pulley2):
#     valid_routes = []
#     invalid_routes = []

#     for route in routes:
#         report = route_collision_report(route, geom, pulley1, pulley2)
#         route["collision_report"] = report
#         route["score"] = score_route(route, geom, pulley1, pulley2)

#         if report["valid"]:
#             valid_routes.append(route)
#         else:
#             invalid_routes.append(route)

#     valid_routes.sort(key=lambda r: r["score"], reverse=True)
#     invalid_routes.sort(key=lambda r: r["route_id"])
#     return valid_routes, invalid_routes


# # ============================================================
# # Plotting
# # ============================================================

# def plot_route_on_axis(ax, route, geom, pulley1, pulley2, title):
#     p1 = geom["p1_2d"]
#     p2 = geom["p2_2d"]
#     c1 = geom["c1_2d"]
#     c2 = geom["c2_2d"]
#     circ1 = geom["circ1"]
#     circ2 = geom["circ2"]

#     t11 = route["seg1"]["tangent_point_2d"]
#     t21 = route["seg2"]["t1_2d"]
#     t22 = route["seg2"]["t2_2d"]
#     t32 = route["seg3"]["tangent_point_2d"]

#     ax.plot(circ1[:, 0], circ1[:, 1])
#     ax.plot(circ2[:, 0], circ2[:, 1])

#     # route segments
#     ax.plot([p1[0], t11[0]], [p1[1], t11[1]], linewidth=2.0)
#     ax.plot([t21[0], t22[0]], [t21[1], t22[1]], linewidth=2.0)
#     ax.plot([t32[0], p2[0]], [t32[1], p2[1]], linewidth=2.0)

#     # radii
#     ax.plot([c1[0], t11[0]], [c1[1], t11[1]], "--", linewidth=1.0)
#     ax.plot([c1[0], t21[0]], [c1[1], t21[1]], "--", linewidth=1.0)
#     ax.plot([c2[0], t22[0]], [c2[1], t22[1]], "--", linewidth=1.0)
#     ax.plot([c2[0], t32[0]], [c2[1], t32[1]], "--", linewidth=1.0)

#     # markers
#     ax.scatter([p1[0], p2[0]], [p1[1], p2[1]], marker="o", s=40)
#     ax.scatter([c1[0], c2[0]], [c1[1], c2[1]], marker="x", s=50)

#     tangent_pts = np.array([t11, t21, t22, t32])
#     ax.scatter(tangent_pts[:, 0], tangent_pts[:, 1], marker="s", s=35)

#     ax.set_title(title)
#     ax.set_xlabel("horizontal")
#     ax.set_ylabel("vertical (+up)")
#     set_equal_limits(ax, np.vstack([circ1, circ2, p1, p2, tangent_pts]))


# def save_valid_routes_grid(valid_routes, geom, pulley1, pulley2, filename):
#     if not valid_routes:
#         print("No valid routes to plot.")
#         return

#     n = len(valid_routes)
#     ncols = min(4, n)
#     nrows = int(np.ceil(n / ncols))

#     fig, axes = plt.subplots(nrows, ncols, figsize=(4.5 * ncols, 4.5 * nrows))
#     axes = np.atleast_2d(axes)

#     for ax in axes.flat[n:]:
#         ax.axis("off")

#     for idx, route in enumerate(valid_routes):
#         ax = axes.flat[idx]
#         title = (
#             f"route {route['route_id']}\n"
#             f"({route['seg1_idx']+1},{route['seg2_idx']+1},{route['seg3_idx']+1})\n"
#             f"score={route['score']:.3f}"
#         )
#         plot_route_on_axis(ax, route, geom, pulley1, pulley2, title)

#     ey = geom["ey"]
#     fig.suptitle(
#         "Valid route combinations\n"
#         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}\n"
#         f"Pulley1={pulley1.direction}, Pulley2={pulley2.direction}",
#         y=1.02
#     )
#     fig.tight_layout()
#     fig.savefig(filename, dpi=220, bbox_inches="tight")
#     plt.close(fig)
#     print(f"Saved {filename}")


# def save_invalid_routes_grid(invalid_routes, geom, pulley1, pulley2, filename):
#     if not invalid_routes:
#         print("No invalid routes to plot.")
#         return

#     n = len(invalid_routes)
#     ncols = min(4, n)
#     nrows = int(np.ceil(n / ncols))

#     fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 4.8 * nrows))
#     axes = np.atleast_2d(axes)

#     for ax in axes.flat[n:]:
#         ax.axis("off")

#     for idx, route in enumerate(invalid_routes):
#         ax = axes.flat[idx]
#         rep = route["collision_report"]
#         title = (
#             f"route {route['route_id']}\n"
#             f"({route['seg1_idx']+1},{route['seg2_idx']+1},{route['seg3_idx']+1})\n"
#             f"bad: "
#             f"{'s1->p2 ' if rep['seg1_hits_pulley2'] else ''}"
#             f"{'mid->p1 ' if rep['seg2_hits_pulley1'] else ''}"
#             f"{'mid->p2 ' if rep['seg2_hits_pulley2'] else ''}"
#             f"{'s3->p1' if rep['seg3_hits_pulley1'] else ''}"
#         )
#         plot_route_on_axis(ax, route, geom, pulley1, pulley2, title.strip())

#     ey = geom["ey"]
#     fig.suptitle(
#         "Rejected route combinations\n"
#         f"selection vertical = +{np.array2string(ey, precision=3, suppress_small=True)}",
#         y=1.02
#     )
#     fig.tight_layout()
#     fig.savefig(filename, dpi=220, bbox_inches="tight")
#     plt.close(fig)
#     print(f"Saved {filename}")


# def save_best_route(best_route, geom, pulley1, pulley2, filename):
#     fig, ax = plt.subplots(figsize=(10, 6))
#     title = (
#         "Chosen globally valid route\n"
#         f"route {best_route['route_id']} | "
#         f"indices=({best_route['seg1_idx']+1},{best_route['seg2_idx']+1},{best_route['seg3_idx']+1}) | "
#         f"score={best_route['score']:.3f}\n"
#         f"selection vertical = +{np.array2string(geom['ey'], precision=3, suppress_small=True)}\n"
#         f"Pulley1={pulley1.direction}, Pulley2={pulley2.direction}"
#     )
#     plot_route_on_axis(ax, best_route, geom, pulley1, pulley2, title)
#     fig.tight_layout()
#     fig.savefig(filename, dpi=220, bbox_inches="tight")
#     plt.close(fig)
#     print(f"Saved {filename}")


# # ============================================================
# # Main example
# # ============================================================

# def save_two_pulley_system_example_fixed(base="example_two_pulley_system_fixed"):
#     point1 = Point3D(center=[-4.0, 0.0, -1.0], name="Point1")
#     point2 = Point3D(center=[10.0, 0.0, 3.5], name="Point2")

#     pulley1 = Pulley(
#         center=[0.0, 0.0, 0.0],
#         radius=1.5,
#         axis=[0.0, 1.0, 0.0],
#         direction="CW",
#         name="Pulley1",
#     )

#     pulley2 = Pulley(
#         center=[5.5, 0.0, 1.5],
#         radius=1.0,
#         axis=[0.0, 1.0, 0.0],
#         direction="CCW",
#         name="Pulley2",
#     )

#     geom = build_common_geometry(point1, point2, pulley1, pulley2)
#     routes = generate_route_candidates(point1, point2, pulley1, pulley2)
#     valid_routes, invalid_routes = filter_and_score_routes(routes, geom, pulley1, pulley2)

#     print(f"Total route combinations: {len(routes)}")
#     print(f"Valid routes: {len(valid_routes)}")
#     print(f"Invalid routes: {len(invalid_routes)}")

#     save_valid_routes_grid(valid_routes, geom, pulley1, pulley2, f"{base}_valid_routes.png")
#     save_invalid_routes_grid(invalid_routes, geom, pulley1, pulley2, f"{base}_invalid_routes.png")

#     if not valid_routes:
#         print("No globally valid route found.")
#         return

#     best = valid_routes[0]
#     save_best_route(best, geom, pulley1, pulley2, f"{base}_chosen.png")

#     t11 = best["seg1"]["tangent_point_2d"]
#     t21 = best["seg2"]["t1_2d"]
#     t22 = best["seg2"]["t2_2d"]
#     t32 = best["seg3"]["tangent_point_2d"]

#     print("Chosen globally valid tangent points:")
#     print("  Point1 -> Pulley1:", t11)
#     print("  Pulley1 -> Pulley2 on Pulley1:", t21)
#     print("  Pulley1 -> Pulley2 on Pulley2:", t22)
#     print("  Pulley2 -> Point2:", t32)


# if __name__ == "__main__":
#     save_two_pulley_system_example_fixed()

import re
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

from rds_finger.config import TENDON_PATH
from rds_finger.statics.plot_tangents import (
    save_point_pulley_all_vs_chosen,
    save_pulley_pulley_all_vs_chosen,
)
from rds_finger.statics.utils import plane_basis_with_vertical, project_to_plane_coords
from rds_finger.statics.tangent import (
    choose_tangent_point_to_pulley,
    choose_tangent_pulley_to_pulley,
)
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.pulleys import Pulley


DEFAULT_OUTPUT_ROOT = Path('tangent_debug_output')


def slugify(value: str) -> str:
    value = re.sub(r'[^A-Za-z0-9._-]+', '_', value.strip())
    value = re.sub(r'_+', '_', value)
    return value.strip('_') or 'item'


def is_point(obj) -> bool:
    return isinstance(obj, Point3D)


def is_pulley(obj) -> bool:
    return isinstance(obj, Pulley)


def element_label(obj) -> str:
    if is_pulley(obj):
        tendon = getattr(obj, 'tendon', 'tendon')
        shaft = getattr(obj, 'shaft', 'shaft')
        name = getattr(obj, 'name', 'pulley')
        return f'pulley_{tendon}_{shaft}_{name}'
    point_type = getattr(obj, 'type', 'POINT')
    tendon = getattr(obj, 'tendon', 'tendon')
    return f'point_{tendon}_{point_type}'


def segment_label(a, b, index: int) -> str:
    a_kind = 'point' if is_point(a) else 'pulley'
    b_kind = 'point' if is_point(b) else 'pulley'
    return f'{index:02d}_{a_kind}_to_{b_kind}_{slugify(element_label(a))}_to_{slugify(element_label(b))}'


def point_pulley_tangent_3d(point: Point3D, pulley: Pulley) -> np.ndarray:
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    origin = np.asarray(pulley.center, dtype=float)
    t2 = np.asarray(choose_tangent_point_to_pulley(point, pulley), dtype=float)
    return origin + t2[0] * ex + t2[1] * ey


def pulley_pulley_tangents_3d(pulley1: Pulley, pulley2: Pulley) -> tuple[np.ndarray, np.ndarray]:
    axis1 = np.asarray(pulley1.axis, dtype=float)
    axis2 = np.asarray(pulley2.axis, dtype=float)
    axis = axis1 if np.dot(axis1, axis2) >= 0 else -axis1
    ex, ey, _ = plane_basis_with_vertical(axis)
    origin = np.asarray(pulley1.center, dtype=float)
    t1_2d, t2_2d = choose_tangent_pulley_to_pulley(pulley1, pulley2)
    t1_2d = np.asarray(t1_2d, dtype=float)
    t2_2d = np.asarray(t2_2d, dtype=float)
    t1_3d = origin + t1_2d[0] * ex + t1_2d[1] * ey
    t2_3d = origin + t2_2d[0] * ex + t2_2d[1] * ey
    return t1_3d, t2_3d


def compute_selected_route_segments(path_items: list) -> tuple[list[dict], dict[int, list[np.ndarray]]]:
    """Return selected route segments and tangent points touching each pulley index."""
    route_segments = []
    pulley_touch_points: dict[int, list[np.ndarray]] = {}

    for idx in range(len(path_items) - 1):
        a = path_items[idx]
        b = path_items[idx + 1]

        if is_point(a) and is_pulley(b):
            tangent = point_pulley_tangent_3d(a, b)
            route_segments.append(
                {
                    'kind': 'point_to_pulley',
                    'index': idx,
                    'start': np.asarray(a.center, dtype=float),
                    'end': tangent,
                    'point': a,
                    'pulley': b,
                }
            )
            pulley_touch_points.setdefault(idx + 1, []).append(tangent)

        elif is_pulley(a) and is_point(b):
            tangent = point_pulley_tangent_3d(b, a)
            route_segments.append(
                {
                    'kind': 'pulley_to_point',
                    'index': idx,
                    'start': tangent,
                    'end': np.asarray(b.center, dtype=float),
                    'point': b,
                    'pulley': a,
                }
            )
            pulley_touch_points.setdefault(idx, []).append(tangent)

        elif is_pulley(a) and is_pulley(b):
            tangent_a, tangent_b = pulley_pulley_tangents_3d(a, b)
            route_segments.append(
                {
                    'kind': 'pulley_to_pulley',
                    'index': idx,
                    'start': tangent_a,
                    'end': tangent_b,
                    'pulley_a': a,
                    'pulley_b': b,
                }
            )
            pulley_touch_points.setdefault(idx, []).append(tangent_a)
            pulley_touch_points.setdefault(idx + 1, []).append(tangent_b)

        else:
            raise TypeError(
                f'Unsupported adjacent path element types at indices {idx} and {idx + 1}: '
                f'{type(a).__name__}, {type(b).__name__}'
            )

    return route_segments, pulley_touch_points


def draw_pulley_circle_3d(ax, pulley: Pulley, num_pts: int = 200, **kwargs) -> None:
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)
    th = np.linspace(0.0, 2.0 * np.pi, num_pts)
    pts = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)


def draw_pulley_wrap_arc_3d(
    ax,
    pulley: Pulley,
    tangent_a: np.ndarray,
    tangent_b: np.ndarray,
    num_pts: int = 120,
    **kwargs,
) -> None:
    """Draw the shorter visible circle arc between two chosen tangent points on one pulley."""
    ex, ey, _ = plane_basis_with_vertical(pulley.axis)
    center = np.asarray(pulley.center, dtype=float)

    def angle_of(pt3):
        rel = np.asarray(pt3, dtype=float) - center
        return np.arctan2(np.dot(rel, ey), np.dot(rel, ex))

    a0 = angle_of(tangent_a)
    a1 = angle_of(tangent_b)
    diff = (a1 - a0 + np.pi) % (2.0 * np.pi) - np.pi
    th = np.linspace(a0, a0 + diff, num_pts)
    pts = (
        center[None, :]
        + pulley.radius * np.cos(th)[:, None] * ex[None, :]
        + pulley.radius * np.sin(th)[:, None] * ey[None, :]
    )
    ax.plot(pts[:, 0], pts[:, 1], pts[:, 2], **kwargs)


def set_axes_equal_3d(ax, points: np.ndarray, pad_frac: float = 0.08) -> None:
    mins = points.min(axis=0)
    maxs = points.max(axis=0)
    centers = 0.5 * (mins + maxs)
    spans = np.maximum(maxs - mins, 1.0)
    radius = 0.5 * spans.max() * (1.0 + pad_frac)

    ax.set_xlim(centers[0] - radius, centers[0] + radius)
    ax.set_ylim(centers[1] - radius, centers[1] + radius)
    ax.set_zlim(centers[2] - radius, centers[2] + radius)


def save_selected_tendon_route_plot(
    tendon_name: str,
    path_items: list,
    output_file: Path,
    elev: float = 24.0,
    azim: float = -58.0,
) -> None:
    route_segments, pulley_touch_points = compute_selected_route_segments(path_items)

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection='3d')

    world_points = []

    for item in path_items:
        center = np.asarray(item.center, dtype=float)
        world_points.append(center)

        if is_point(item):
            ax.scatter(center[0], center[1], center[2], marker='o', s=42)
            ax.text(center[0], center[1], center[2], f' {getattr(item, "type", "PT")}', fontsize=8)
        else:
            draw_pulley_circle_3d(ax, item, linewidth=1.2)
            ax.scatter(center[0], center[1], center[2], marker='x', s=48)
            ax.text(center[0], center[1], center[2], f' {getattr(item, "shaft", "?")}/{getattr(item, "name", "pulley")}', fontsize=8)

    for seg in route_segments:
        s = np.asarray(seg['start'], dtype=float)
        e = np.asarray(seg['end'], dtype=float)
        world_points.extend([s, e])
        ax.plot([s[0], e[0]], [s[1], e[1]], [s[2], e[2]], linewidth=2.4)
        ax.scatter([s[0], e[0]], [s[1], e[1]], [s[2], e[2]], marker='s', s=20)

    for pulley_idx, tangents in pulley_touch_points.items():
        if len(tangents) >= 2:
            draw_pulley_wrap_arc_3d(
                ax,
                path_items[pulley_idx],
                tangents[0],
                tangents[1],
                linewidth=1.2,
                linestyle='--',
            )
            world_points.extend(tangents[:2])

    world_points = np.asarray(world_points, dtype=float)
    set_axes_equal_3d(ax, world_points)
    ax.view_init(elev=elev, azim=azim)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')
    ax.set_title(f'Selected tangent path: {tendon_name}')
    fig.tight_layout()
    fig.savefig(output_file, dpi=220, bbox_inches='tight')
    plt.close(fig)


def save_pairwise_segment_plots_for_tendon(
    tendon_name: str,
    path_items: list,
    pairwise_dir: Path,
) -> list[Path]:
    saved_files: list[Path] = []

    for idx in range(len(path_items) - 1):
        a = path_items[idx]
        b = path_items[idx + 1]
        file_base = segment_label(a, b, idx)
        out_file = pairwise_dir / f'{file_base}_all_vs_selected.png'

        if is_point(a) and is_pulley(b):
            save_point_pulley_all_vs_chosen(a, b, str(out_file))
        elif is_pulley(a) and is_point(b):
            save_point_pulley_all_vs_chosen(b, a, str(out_file))
        elif is_pulley(a) and is_pulley(b):
            save_pulley_pulley_all_vs_chosen(a, b, str(out_file))
        else:
            raise TypeError(
                f'Unsupported adjacent path element types for {tendon_name} segment {idx}: '
                f'{type(a).__name__}, {type(b).__name__}'
            )

        saved_files.append(out_file)

    return saved_files


def save_tendon_debug_bundle(
    tendon_name: str,
    path_items: list,
    output_root: Path = DEFAULT_OUTPUT_ROOT,
) -> dict:
    tendon_dir = output_root / slugify(tendon_name)
    pairwise_dir = tendon_dir / 'pairwise_segments'
    combined_dir = tendon_dir / 'combined_route'
    pairwise_dir.mkdir(parents=True, exist_ok=True)
    combined_dir.mkdir(parents=True, exist_ok=True)

    pairwise_files = save_pairwise_segment_plots_for_tendon(tendon_name, path_items, pairwise_dir)
    combined_file = combined_dir / f'{slugify(tendon_name)}_selected_route_3d.png'
    save_selected_tendon_route_plot(tendon_name, path_items, combined_file)

    return {
        'tendon': tendon_name,
        'tendon_dir': tendon_dir,
        'pairwise_dir': pairwise_dir,
        'combined_dir': combined_dir,
        'pairwise_files': pairwise_files,
        'combined_file': combined_file,
    }


def save_all_tendon_debug_plots(
    tendon_paths: dict | None = None,
    output_root: Path | str = DEFAULT_OUTPUT_ROOT,
) -> list[dict]:
    if tendon_paths is None:
        tendon_paths = TENDON_PATH

    output_root = Path(output_root)
    output_root.mkdir(parents=True, exist_ok=True)

    reports = []
    for tendon_name, path_items in tendon_paths.items():
        report = save_tendon_debug_bundle(tendon_name, path_items, output_root=output_root)
        reports.append(report)
        print(f'Saved {tendon_name}:')
        print(f'  pairwise -> {report["pairwise_dir"]}')
        print(f'  combined -> {report["combined_file"]}')

    return reports


if __name__ == '__main__':
    save_all_tendon_debug_plots()
