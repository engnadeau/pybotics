"""Pybotics modules."""
import logging

from pkg_resources import get_distribution, DistributionNotFound

# setup version info
try:
    __version__ = get_distribution(__name__).version
except DistributionNotFound as e:
    # package is not installed
    pass

# import modules
from .kinematic_chain import KinematicChain
from .robot import Robot
from .tool import Tool

# set logging
logging.getLogger(__name__).addHandler(logging.NullHandler())
