"""Pybotics errors."""
from typing import Optional

from pybotics.kinematic_pair import KinematicPair
from pybotics.link_convention import LinkConvention
from pybotics.orientation_convention import OrientationConvention


class PyboticsError(Exception):
    """Base class for Pybotics errors."""

    _default_message = 'Pybotics error'

    def __init__(self, message: Optional[str] = None) -> None:
        """
        Construct base exception.

        :param message:
        """
        self.message = message

    def __str__(self) -> str:
        """
        Convert exception to string.

        :return: string
        """
        if self.message is None:
            return self._default_message
        else:
            return self.message


class KinematicPairError(PyboticsError):
    """Inappropriate KinematicPair."""

    _default_message = 'Supported pairs: {}'.format(
        [e.name for e in KinematicPair])


class LinkConventionError(PyboticsError):
    """Inappropriate LinkConvention."""

    _default_message = 'Supported conventions: {}'.format(
        [e.name for e in LinkConvention])


class LinkSequenceError(PyboticsError):
    """Multiple conventions in link sequence."""

    _default_message = 'All links must use the same convention'


class Matrix4x4Error(PyboticsError):
    """Inappropriate matrix value."""

    def __init__(self, name: str) -> None:
        """
        Construct exception.

        :param name: name of parameter causing error
        """
        message = '{} must be a 4x4 ndarray'.format(name)
        super().__init__(message)


class OrientationConventionError(PyboticsError):
    """Inappropriate OrientationConvention."""

    _default_message = 'Supported conventions: {}'.format(
        [e.name for e in OrientationConvention])


class SequenceError(PyboticsError):
    """Inappropriate sequence length."""

    def __init__(self, name: str, required_length: int) -> None:
        """
        Construct exception.

        :param name: name of parameter causing error
        :param required_length: required length of parameter
        """
        message = '{} must be a 1D sequence with len=={}'.format(
            name, required_length)
        super().__init__(message)


class ShapeMismatchError(PyboticsError):
    """Inappropriate shapes for two paired sequences."""

    def __init__(self, name_a: str, name_b: str) -> None:
        """
        Construct exception.

        :param name_a: name of parameter causing error
        :param name_b: name of parameter causing error
        """
        message = '{} and {} must be the same size'.format(name_a,
                                                           name_b)
        super().__init__(message)
