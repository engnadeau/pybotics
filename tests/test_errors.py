"""Test errors."""

from pytest import raises

from pybotics.errors import PyboticsError


def test_errors():
    """
    Test error.

    :return:
    """
    with raises(PyboticsError):
        raise PyboticsError()

    assert str(PyboticsError()) == PyboticsError().message
    assert str(PyboticsError("test")) == "test"
