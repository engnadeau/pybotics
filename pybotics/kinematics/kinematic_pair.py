from enum import Enum, unique, auto


@unique
class KinematicPair(Enum):
    UNDEFINED = auto(),
    REVOLUTE = auto(),
    PRISMATIC = auto()
