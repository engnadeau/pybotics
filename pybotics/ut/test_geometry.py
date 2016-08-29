from unittest import TestCase
import numpy as np

from pybotics.geometry import xyzrpw_2_pose


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
