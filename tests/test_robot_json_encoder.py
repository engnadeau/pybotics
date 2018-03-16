"""Test robot JSON encoder."""
from pytest import raises

from pybotics.robot import RobotJSONEncoder


def test_default():
    encoder = RobotJSONEncoder()

    with raises(TypeError):
        encoder.encode({1, 2})
