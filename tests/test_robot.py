from unittest import TestCase
import logging
from pybotics import robot_model
from pybotics.robot import Robot
import numpy as np


class TestRobot(TestCase):
    def setUp(self):
        logging.basicConfig(level=logging.DEBUG, format='%(asctime)s:\t%(message)s')
        np.set_printoptions(suppress=True)
        self.robot = Robot(robot_model.ur10())

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

        # test single transform
        actual_transform = self.robot.fk(joints)

        assert actual_transform.shape[0] == expected_transform.shape[0]
        assert actual_transform.shape[1] == expected_transform.shape[1]
        assert actual_transform.size == expected_transform.size
        np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)

    def test_impair_robot_model(self):
        impaired_robot = Robot(robot_model.ur10())
        impaired_robot.impair_robot_model(0.1)

        model_diff = impaired_robot.robot_model - self.robot.robot_model

        for link_parameters in model_diff:
            for parameter in link_parameters:
                assert abs(parameter) > 1e-9

    def test_ik(self):
        expected_joints_list = np.deg2rad([
            [0, -90, 90, 0, 90, 0],
            [10, 90, 80, 20, 90, 123],
            [20, -40, 90, 10, 20, 0],
            [-10, 20, -30, 40, -50, 60]
        ])

        for expected_joints in expected_joints_list:
            # define test values

            expected_transform = self.robot.fk(expected_joints)

            # test single transform
            actual_joints = self.robot.ik(expected_transform)
            actual_transform = self.robot.fk(actual_joints)

            assert len(actual_joints) == len(expected_joints)
            np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-1, atol=1e-1)
