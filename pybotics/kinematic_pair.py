"""Kinematic pair module."""
from enum import IntEnum, unique


@unique
class KinematicPair(IntEnum):
    """
    Kinematic pair.

    Connection between bodies with relative movement constraints.
    """

    UNDEFINED = 0,
    REVOLUTE = 1,
    PRISMATIC = 2
