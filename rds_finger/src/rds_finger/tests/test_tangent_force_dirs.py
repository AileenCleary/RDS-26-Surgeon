import numpy as np

from rds_finger.config import CW, CCW
from rds_finger.statics.tangent import (
    segment_dir,
    force_dir_on_pulley_from_prev,
    force_dir_on_pulley_to_next,
)
from rds_finger.types.fixed_points import Point3D
from rds_finger.types.pulleys import Pulley


def assert_unit(v, tol=1e-9):
    n = np.linalg.norm(v)
    assert abs(n - 1.0) < tol, f"Vector not unit length: {v}, norm={n}"


def assert_opposite(v1, v2, tol=1e-6):
    dot = float(np.dot(v1, v2))
    assert abs(dot + 1.0) < tol, f"Vectors not opposite.\n v1={v1}\n v2={v2}\n dot={dot}"


def assert_same(v1, v2, tol=1e-6):
    dot = float(np.dot(v1, v2))
    assert abs(dot - 1.0) < tol, f"Vectors not same.\n v1={v1}\n v2={v2}\n dot={dot}"


def test_shared_segment_between_two_pulleys_gives_opposite_forces():
    p1 = Pulley(
        center=np.array([0.0, 0.0, 0.0]),
        radius=1.0,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="S1",
        name="P1",
    )

    p2 = Pulley(
        center=np.array([6.0, 0.0, 0.0]),
        radius=1.0,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="S2",
        name="P2",
    )

    f_on_p1_to_p2 = force_dir_on_pulley_to_next(p1, p2)
    f_on_p2_from_p1 = force_dir_on_pulley_from_prev(p1, p2)

    assert_unit(f_on_p1_to_p2)
    assert_unit(f_on_p2_from_p1)
    assert_opposite(f_on_p1_to_p2, f_on_p2_from_p1)


def test_segment_dir_matches_forward_path_for_two_pulleys():
    p1 = Pulley(
        center=np.array([0.0, 0.0, 0.0]),
        radius=1.0,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="S1",
        name="P1",
    )

    p2 = Pulley(
        center=np.array([6.0, 0.0, 0.0]),
        radius=1.0,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="S2",
        name="P2",
    )

    d12 = segment_dir(p1, p2)
    d21 = segment_dir(p2, p1)

    assert_unit(d12)
    assert_unit(d21)
    assert_opposite(d12, d21)


def test_point_pulley_segment_path_and_force_are_opposite():
    pt = Point3D(
        center=np.array([6.0, 0.0, 2.0]),
        type="START",
        tendon="flexor",
    )

    pulley = Pulley(
        center=np.array([0.0, 0.0, 0.0]),
        radius=1.5,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="MCP",
        name="PulleyA",
    )

    d_path = segment_dir(pt, pulley)
    f_on_pulley = force_dir_on_pulley_from_prev(pt, pulley)

    assert_unit(d_path)
    assert_unit(f_on_pulley)
    assert_opposite(d_path, f_on_pulley)


def test_pulley_point_segment_path_and_force_are_same_from_pulley_perspective():
    pulley = Pulley(
        center=np.array([0.0, 0.0, 0.0]),
        radius=1.5,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="MCP",
        name="PulleyA",
    )

    pt = Point3D(
        center=np.array([6.0, 0.0, 2.0]),
        type="END",
        tendon="flexor",
    )

    d_path = segment_dir(pulley, pt)
    f_on_pulley = force_dir_on_pulley_to_next(pulley, pt)

    assert_unit(d_path)
    assert_unit(f_on_pulley)
    assert_same(d_path, f_on_pulley)


def test_symmetric_point_pulley_point_has_symmetric_pulley_forces():
    left = Point3D(
        center=np.array([-8.0, 0.0, 3.0]),
        type="START",
        tendon="flexor",
    )

    right = Point3D(
        center=np.array([8.0, 0.0, 3.0]),
        type="END",
        tendon="flexor",
    )

    pulley = Pulley(
        center=np.array([0.0, 0.0, 0.0]),
        radius=1.0,
        axis=np.array([0.0, 1.0, 0.0]),
        dir=CW,
        tendon="flexor",
        shaft="MCP",
        name="MidPulley",
    )

    v_in = force_dir_on_pulley_from_prev(left, pulley)
    v_out = force_dir_on_pulley_to_next(pulley, right)

    assert_unit(v_in)
    assert_unit(v_out)

    # symmetry: x components opposite, z components same
    assert np.isclose(v_in[0], -v_out[0], atol=1e-6), f"x-components not opposite: {v_in}, {v_out}"
    assert np.isclose(v_in[2],  v_out[2], atol=1e-6), f"z-components not equal: {v_in}, {v_out}"

    # resulting net force should have near-zero x and positive z
    f_net = v_in + v_out
    assert abs(f_net[0]) < 1e-6, f"Expected x cancellation, got f_net={f_net}"
    assert f_net[2] > 0.0, f"Expected positive z resultant, got f_net={f_net}"

if __name__ == "__main__":
    test_shared_segment_between_two_pulleys_gives_opposite_forces()
    test_segment_dir_matches_forward_path_for_two_pulleys()
    test_point_pulley_segment_path_and_force_are_opposite()
    test_pulley_point_segment_path_and_force_are_same_from_pulley_perspective()
    test_symmetric_point_pulley_point_has_symmetric_pulley_forces()

    print("All tangent/force-direction tests passed.")