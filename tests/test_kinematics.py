from unittest import TestCase
import numpy as np
import math

from pybotics.kinematics import forward_transform


class TestKinematics(TestCase):
    def test_forward_transform(self):
        mdh = [math.pi, 100, math.pi / 2, 200]

        expected_transform = np.array([
            [0, - 1, 0, 100],
            [-1, 0, 0, 0],
            [0, 0, -1, -200],
            [0, 0, 0, 1]
        ])

        actual_transform = forward_transform(mdh)
        np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)
