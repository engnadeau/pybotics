from unittest import TestCase

from pybotics import robot_model


class TestUr10(TestCase):
    def test_ur10(self):
        model = robot_model.ur10()

        assert model.shape[0] == 6
        assert model.shape[1] == 4

    def test_puma_560(self):
        model = robot_model.puma_560()

        assert model.shape[0] == 6
        assert model.shape[1] == 4

    def test_kuka_lbr_iiwa_7(self):
        model = robot_model.kuka_lbr_iiwa_7()

        assert model.shape[0] == 7
        assert model.shape[1] == 4
