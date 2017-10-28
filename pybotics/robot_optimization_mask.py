"""Robot optimization mask module."""
from typing import NamedTuple, Union, Sequence

# TODO: move to Python 3.6 style one day...
# https://docs.python.org/3/library/typing.html#typing.NamedTuple
RobotOptimizationMask = NamedTuple(
    'RobotOptimizationMask',
    [
        ('world_frame', Union[bool, Sequence[bool]]),
        ('kinematic_chain', Union[bool, Sequence[bool]]),
        ('tool', Union[bool, Sequence[bool]])
    ]
)
