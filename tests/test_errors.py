"""Test errors."""
from pytest import raises

from pybotics.errors import PyboticsError, SequenceLengthError, Matrix4x4Error, \
    LinkConventionError, LinkSequenceError, OrientationConventionError


def test_pybotics_error():
    with raises(PyboticsError):
        raise PyboticsError()


def test_sequence_length_error():
    with raises(SequenceLengthError):
        raise SequenceLengthError('test', 123)


def test_matrix_4x4_error():
    with raises(Matrix4x4Error):
        raise Matrix4x4Error('test')


def test_link_convention_error():
    with raises(LinkConventionError):
        raise LinkConventionError()


def test_link_sequence_error():
    with raises(LinkSequenceError):
        raise LinkSequenceError()


def test_orientation_convention_error():
    with raises(OrientationConventionError):
        raise OrientationConventionError()
