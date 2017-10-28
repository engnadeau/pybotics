"""Test errors."""
from pytest import raises

from pybotics.errors import PyboticsError, SequenceLengthError, Matrix4x4Error, \
    LinkConventionError, LinkSequenceError, OrientationConventionError


def test_pybotics_error():
    with raises(PyboticsError):
        raise PyboticsError()

    assert str(PyboticsError()) is PyboticsError._default_message
    assert str(PyboticsError('test')) is 'test'


def test_link_convention_error():
    with raises(LinkConventionError):
        raise LinkConventionError()


def test_link_sequence_error():
    with raises(LinkSequenceError):
        raise LinkSequenceError()


def test_matrix_4x4_error():
    with raises(Matrix4x4Error):
        raise Matrix4x4Error('test')


def test_orientation_convention_error():
    with raises(OrientationConventionError):
        raise OrientationConventionError()


def test_sequence_length_error():
    with raises(SequenceLengthError):
        raise SequenceLengthError('test', 123)
