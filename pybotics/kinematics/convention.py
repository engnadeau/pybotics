from enum import Enum, unique, auto


@unique
class Convention(Enum):
    UNDEFINED = auto()
    MDH = auto(),
    DH = auto()
