"""Orientation convention module."""
from enum import Enum, unique


@unique
class OrientationConvention(Enum):
    """Orientation of a body with respect to a fixed coordinate system."""

    EULER_ZYX = 0
