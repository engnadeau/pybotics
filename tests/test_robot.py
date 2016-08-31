from unittest import TestCase

from pybotics import robot_model
from pybotics.robot import Robot


class TestRobot(TestCase):
    def setUp(self):
        self.robot = Robot()
        self.robot.robot_model = robot_model.ur10()

    def test_num_dof(self):
        assert self.robot.num_dof() == 6

    def test_fk(self):
        self.fail()

    def test_impair_robot_model(self):
        self.fail()
