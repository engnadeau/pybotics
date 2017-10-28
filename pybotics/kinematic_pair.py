"""Kinematic pair module."""
from enum import IntEnum, unique


@unique
class KinematicPair(IntEnum):
    """
    Kinematic pair.

    Connection between bodies with relative movement constraints.
    """

    REVOLUTE = 0
    # TODO: add PRISMATIC
