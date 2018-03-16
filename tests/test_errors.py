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

    assert str(PyboticsError()) is PyboticsError._default_message
    assert str(PyboticsError('test')) is 'test'
