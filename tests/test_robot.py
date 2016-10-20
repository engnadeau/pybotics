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
        joints = [0, -90, 90, 0, 90, 0]

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

        # test multiple joint configs
        num_configs = 10
        joint_list = [joints] * num_configs

        actual_transforms = self.robot.fk(joint_list)

        assert len(actual_transforms) == num_configs

        for actual_transform in actual_transforms:
            assert actual_transform.shape[0] == expected_transform.shape[0]
            assert actual_transform.shape[1] == expected_transform.shape[1]
            assert actual_transform.size == expected_transform.size
            np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-6, atol=1e-6)


            # TODO: test up to n-th joint

    def test_impair_robot_model(self):
        impaired_robot = Robot(robot_model.ur10())
        impaired_robot.impair_robot_model(0.1)

        model_diff = impaired_robot.robot_model - self.robot.robot_model

        for link_parameters in model_diff:
            for parameter in link_parameters:
                assert abs(parameter) > 1e-9

    def test_ik(self):
        for i in range(10):
            # define test values
            expected_joints = np.deg2rad([0, -90, 90, 0, 90, 0])

            if i != 0:
                expected_joints += np.random.rand(expected_joints.size) - 0.5

            expected_transform = self.robot.fk(expected_joints)

            # test single transform
            actual_joints = self.robot.ik(expected_transform)
            actual_transform = self.robot.fk(actual_joints)

            logging.info('Iteration: %d', i)
            logging.info('Expected Joints: ' + np.array_repr(np.rad2deg(expected_joints)))

            assert len(actual_joints) == len(expected_joints)
            np.testing.assert_allclose(actual=actual_transform, desired=expected_transform, rtol=1e-1, atol=1e-1)
