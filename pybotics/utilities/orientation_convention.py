from enum import Enum, unique, auto


@unique
class OrientationConvention(Enum):
    EULER_ZYX = auto()
