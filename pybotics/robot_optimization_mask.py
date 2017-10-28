"""Robot optimization mask module."""
from typing import NamedTuple, Union, Sequence


class RobotOptimizationMask(NamedTuple):
    """Optimization mask used in the robot class."""

    world_frame: Union[bool, Sequence[bool]] = False
    kinematic_chain: Union[bool, Sequence[bool]] = False
    tool: Union[bool, Sequence[bool]] = False
