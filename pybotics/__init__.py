"""Pybotics modules."""
import logging

from pkg_resources import get_distribution, DistributionNotFound

# setup version info
try:
    __version__ = get_distribution(__name__).version
except DistributionNotFound:  # pragma: no cover
    # package is not installed
    pass

# import modules
from .kinematic_chain import KinematicChain
from .robot import Robot
from .tool import Tool

__all__ = ['KinematicChain', 'Robot', 'Tool']

# set logging
logging.getLogger(__name__).addHandler(logging.NullHandler())
