from enum import unique, IntEnum


@unique
class Convention(IntEnum):
    UNDEFINED = 0
    MDH = 1,
    DH = 2
