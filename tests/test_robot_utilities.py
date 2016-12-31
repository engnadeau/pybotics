from unittest import TestCase
import numpy as np

from pybotics import robot_utilities


class TestKinematics(TestCase):
    def test_random_joints(self):
        test_angle_limits = [(-np.pi, np.pi)] * 6
        result_joints = robot_utilities.random_joints(test_angle_limits)

        self.assertEqual(len(test_angle_limits), len(result_joints))

        for limit, joint in zip(test_angle_limits, result_joints):
            self.assertTrue(min(limit) < joint)
            self.assertTrue(max(limit) > joint)
