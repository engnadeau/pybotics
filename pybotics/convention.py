from enum import unique, Enum


@unique
class Convention(Enum):
    UNDEFINED = 0
    MDH = 1,
    DH = 2
