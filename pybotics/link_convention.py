"""Link convention module."""
from enum import Enum


class LinkConvention(Enum):
    """Conventions for attaching frames to the links of a kinematic chain."""

    UNDEFINED = 0
    MDH = 4,
