from unittest import TestCase

import math
import numpy as np

from pybotics.geometry import xyzrpw_2_pose, pose_2_xyzrpw


class TestGeometry(TestCase):
    def test_xyzrpw_2_pose(self):
        xyzrpw = [100, 200, 300, -30, 50, 90]

        expected_transform = np.array([
            [0, -0.642788, 0.766044, 100],
            [0.866025, 0.383022, 0.321394, 200],
            [-0.5, 0.663414, 0.556670, 300],
            [0, 0, 0, 1]
        ])

        actual_transform = xyzrpw_2_pose(xyzrpw)
        np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)

    def test_xyzrpw_2_pose_2_xyzrpw_2_pose(self):
        """
        We can only compare 4x4 transform matrices, rpw combinations are arbitrary.
        """
        x = 1.1
        y = 2.2
        z = 3.3

        angles = np.linspace(-math.pi, math.pi, 25)

        # test radians
        for r in angles:
            for p in angles:
                for w in angles:
                    xyzrpw_original = [x, y, z, r, p, w]
                    pose_original = xyzrpw_2_pose(xyzrpw_original, is_radians=True)

                    xyzrpw_result = pose_2_xyzrpw(pose_original, is_radians=True)
                    pose_result = xyzrpw_2_pose(xyzrpw_result, is_radians=True)

                    np.testing.assert_allclose(actual=pose_original, desired=pose_result, rtol=1e-6, atol=1e-6)

        angles = np.rad2deg(angles)
        # test degrees
        for r in angles:
            for p in angles:
                for w in angles:
                    xyzrpw_original = [x, y, z, r, p, w]
                    pose_original = xyzrpw_2_pose(xyzrpw_original)

                    xyzrpw_result = pose_2_xyzrpw(pose_original)
                    pose_result = xyzrpw_2_pose(xyzrpw_result)

                    np.testing.assert_allclose(actual=pose_original, desired=pose_result, rtol=1e-6, atol=1e-6)
