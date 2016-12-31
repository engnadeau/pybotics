from unittest import TestCase
from pybotics.robot import Robot
import numpy as np


class TestRobot(TestCase):
    def setUp(self):
        # load UR10 model for testing
        model = np.array([
            0, 0, 0, 118,
            1.5707963267949, 0, 3.14159265358979, 0,
            0, 612.7, 0, 0,
            0, 571.6, 0, 163.9,
            - 1.5707963267949, 0, 0, 115.7,
            1.5707963267949, 0, 3.14159265358979, 92.2
        ]).reshape((-1, 4))
        self.robot = Robot(model)

    def test_num_dof(self):
        # ur10 has 6 DOF
        assert self.robot.num_dof() == 6

    def test_fk(self):
        # define test values
        joints = np.deg2rad([0, -90, 90, 0, 90, 0])

        expected_transform = np.array([
            [0, 0, -1, -663.8],
            [-1, 0, -0, -163.9],
            [-0, 1, 0, 615],
            [0, 0, 0, 1]
        ])

        # test fk
        reference_frames = [None, np.eye(4)]
        torques = [None, [0] * self.robot.num_dof()]
        for reference_frame in reference_frames:
            for torque in torques:
                actual_transform = self.robot.fk(joints, reference_frame=reference_frame, torques=torque)

                assert actual_transform.shape[0] == expected_transform.shape[0]
                assert actual_transform.shape[1] == expected_transform.shape[1]
                assert actual_transform.size == expected_transform.size
                np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)

        # test exception raising
        with self.assertRaises(AssertionError):
            self.robot.fk(joints, torques=[0] * (self.robot.num_dof() + 1))

    def test_ik(self):
        test_joints_list = np.deg2rad([
            [0, -90, 90, 0, 90, 0],
            [10, 90, 80, 20, 90, 123],
            [20, -40, 90, 10, 20, 0],
            [-10, 20, -30, 40, -50, 60]
        ])

        for test_joints in test_joints_list:
            # define test values
            expected_transform = self.robot.fk(test_joints)

            # test single transform
            ik_joints = self.robot.ik(expected_transform)
            result_transform = self.robot.fk(ik_joints)

            assert len(ik_joints) == len(test_joints)
            np.testing.assert_allclose(actual=result_transform, desired=expected_transform, rtol=1e-1, atol=1e-1)

    def test_validate_joint_angles(self):
        test_joints_list = np.deg2rad([
            [10, 90, 80, 20, 90, 123],
            [-10, 20, -30, 40, -50, 191]
        ])

        self.assertTrue(self.robot.validate_joint_angles(test_joints_list[0]))
        self.assertFalse(self.robot.validate_joint_angles(test_joints_list[1]))
