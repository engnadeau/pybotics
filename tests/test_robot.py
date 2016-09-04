from unittest import TestCase

from pybotics import robot_model
from pybotics.robot import Robot
import numpy as np


class TestRobot(TestCase):
    def setUp(self):
        self.robot = Robot()
        self.robot.robot_model = robot_model.ur10()

    def test_num_dof(self):
        # ur10 has 6 DOF
        assert self.robot.num_dof() == 6

    def test_fk_single(self):
        transform = self.robot.fk()

        # assert 4x4 transform
        assert transform.shape[0] == 4
        assert transform.shape[1] == 4
        assert transform.size == 16

    def test_fk_list(self):

        # number of configs to test
        num_configs = 10

        # list of joint configs
        joint_list = np.random.rand(num_configs, self.robot.num_dof())

        transforms = self.robot.fk(joint_list)

        # assert same number out as in
        assert len(transforms) == num_configs

        # assert 4x4 transform
        for transform in transforms:
            assert transform.shape[0] == 4
            assert transform.shape[1] == 4
            assert transform.size == 16

    def test_impair_robot_model(self):
        impaired_robot = Robot()
        impaired_robot.robot_model = robot_model.ur10()
        impaired_robot.impair_robot_model(0.1)

        model_diff = impaired_robot.robot_model - self.robot.robot_model

        for link_parameters in model_diff:
            for parameter in link_parameters:
                assert abs(parameter) > 1e-9
