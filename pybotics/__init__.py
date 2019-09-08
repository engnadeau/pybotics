"""Pybotics modules."""
from pkg_resources import DistributionNotFound, get_distribution

# import modules
from .kinematic_chain import KinematicChain
from .robot import Robot
from .tool import Tool

# setup version info
try:
    __version__ = get_distribution(__name__).version
except DistributionNotFound:  # pragma: no cover
    # package is not installed
    pass


__all__ = ["KinematicChain", "Robot", "Tool"]
