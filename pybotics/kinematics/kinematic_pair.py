from enum import IntEnum, unique


@unique
class KinematicPair(IntEnum):
    UNDEFINED = 0,
    REVOLUTE = 1,
    PRISMATIC = 2
