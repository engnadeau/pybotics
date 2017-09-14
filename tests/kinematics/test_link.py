from pytest import raises

from pybotics.kinematics.link import Link


def test_link():
    with raises(TypeError):
        Link()
