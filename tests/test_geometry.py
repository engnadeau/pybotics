import math
import numpy as np

from pybotics.geometry import xyzrpw_2_pose, pose_2_xyzrpw, wrap_2_pi


def test_xyzrpw_2_pose():
    xyzrpw = [100, 200, 300, np.deg2rad(-30), np.deg2rad(50), np.deg2rad(90)]

    expected_transform = np.array([
        [0, -0.642788, 0.766044, 100],
        [0.866025, 0.383022, 0.321394, 200],
        [-0.5, 0.663414, 0.556670, 300],
        [0, 0, 0, 1]
    ])

    actual_transform = xyzrpw_2_pose(xyzrpw)
    np.testing.assert_allclose(actual=actual_transform,
                               desired=expected_transform,
                               rtol=1e-6, atol=1e-6)


def test_xyzrpw_2_pose_2_xyzrpw_2_pose():
    """We can only compare 4x4 transform matrices, rpw combinations are arbitrary."""
    x = 1.1
    y = 2.2
    z = 3.3

    angles = np.linspace(-math.pi, math.pi, 25)

    for r in angles:
        for p in angles:
            for w in angles:
                xyzrpw_original = [x, y, z, r, p, w]
                pose_original = xyzrpw_2_pose(xyzrpw_original)

                xyzrpw_result = pose_2_xyzrpw(pose_original)
                pose_result = xyzrpw_2_pose(xyzrpw_result)

                np.testing.assert_allclose(actual=pose_original, desired=pose_result, rtol=1e-6, atol=1e-6)


def test_wrap_2_pi():
    angles = np.array([
        [0, 0],
        [-np.pi, -np.pi],
        [np.pi, -np.pi],
        [2 * np.pi, 0],
        [-2 * np.pi, 0]
    ])

    test_angles = angles[:, 0]
    expected_angles = angles[:, 1]

    # test whole array
    actual_angles = wrap_2_pi(test_angles)
    assert len(test_angles) == len(expected_angles)
    assert len(actual_angles) == len(expected_angles)
    np.testing.assert_allclose(actual_angles, expected_angles)

    # test single elements
    for i, _ in enumerate(expected_angles):
        actual_angle = wrap_2_pi(test_angles[i])
        np.testing.assert_allclose(actual_angle, expected_angles[i])
