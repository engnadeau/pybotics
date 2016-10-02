from unittest import TestCase
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

    def test_xyzrpw_2_pose_2_xyzrpw(self):
        xyzrpw_original = [100, 200, 300, -30, 50, 90]
        pose = xyzrpw_2_pose(xyzrpw_original)
        xyzrpw_result = pose_2_xyzrpw(pose)

        np.testing.assert_allclose(actual=xyzrpw_result, desired=xyzrpw_original, rtol=1e-6, atol=1e-6)
