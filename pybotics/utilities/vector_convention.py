from enum import Enum, unique, auto


@unique
class VectorConvention(Enum):
    EULER_ZYX = auto()
