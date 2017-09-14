import numpy as np
import os

from pytest import fixture

from pybotics.kinematics.kinematic_chain import KinematicChain
from pybotics.models.robot import Robot

np.set_printoptions(suppress=True)

ROBOT_MDH_PATH = os.path.abspath(os.path.join(
    os.path.dirname(__file__),
    os.pardir,
    os.pardir,
    'robot-models',
    'ur10-mdh.csv'
))


@fixture(name='robot')
def robot_fixture():
    kc = KinematicChain.from_revolute_mdh(np.loadtxt(ROBOT_MDH_PATH, delimiter=','))
    return Robot(kc)
