from enum import Enum, unique, auto


@unique
class Convention(Enum):
    REVOLUTE_MDH = auto()
