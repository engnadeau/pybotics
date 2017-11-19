"""Pybotics modules."""
from . import calibration
from . import constants
from . import errors
from . import geometry
from . import validation
from .frame import Frame
from .kinematic_chain import KinematicChain
from .kinematic_pair import KinematicPair
from .link import Link
from .link_convention import LinkConvention
from .mdh_link import MDHLink
from .orientation_convention import OrientationConvention
from .revolute_mdh_link import RevoluteMDHLink
from .robot import Robot
from .robot_optimization_mask import RobotOptimizationMask
from .tool import Tool

__all__ = [
    'calibration',
    'constants',
    'errors',
    'Frame',
    'geometry',
    'KinematicChain',
    'KinematicPair',
    'Link',
    'LinkConvention',
    'MDHLink',
    'OrientationConvention',
    'RevoluteMDHLink',
    'Robot',
    'RobotOptimizationMask',
    'Tool',
    'validation',
]

with open('VERSION') as f:
    __version__ = f.read()
