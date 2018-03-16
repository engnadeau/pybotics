"""Orientation convention module."""
from enum import Enum, unique, IntEnum


class Orientation(Enum):
    """Orientation of a body with respect to a fixed coordinate system."""

    EULER_XYX = 'xyx'
    EULER_XYZ = 'xyz'
    EULER_XZX = 'xzx'
    EULER_XZY = 'xzy'
    EULER_YXY = 'yxy'
    EULER_YXZ = 'yxz'
    EULER_YZX = 'yzx'
    EULER_YZY = 'yzy'
    EULER_ZXY = 'zxy'
    EULER_ZXZ = 'zxz'
    EULER_ZYX = 'zyx'
    EULER_ZYZ = 'zyz'

    FIXED_XYX = 'xyx'
    FIXED_XYZ = 'zyx'
    FIXED_XZX = 'xzx'
    FIXED_XZY = 'yzx'
    FIXED_YXY = 'yxy'
    FIXED_YXZ = 'zxy'
    FIXED_YZX = 'xzy'
    FIXED_YZY = 'yzy'
    FIXED_ZXY = 'yxz'
    FIXED_ZXZ = 'zxz'
    FIXED_ZYX = 'xyz'
    FIXED_ZYZ = 'zyz'


class Link(IntEnum):
    """Conventions for attaching frames to the links of a kinematic chain."""

    MDH = 4


@unique
class KinematicPair(IntEnum):
    """
    Kinematic pair.

    Connection between bodies with relative movement constraints.
    """

    REVOLUTE = 0
