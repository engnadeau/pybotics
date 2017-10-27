"""Pytest config."""
import os

import numpy as np
from pytest import fixture

from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot

np.set_printoptions(suppress=True)


@fixture(name='serial_robot')
def robot_fixture():
    """
    Generate serial robot model.

    :return:
    """
    serial_robot_mdh_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        os.pardir,
        os.pardir,
        'robot-models',
        'ur10-mdh.csv'))

    array = np.loadtxt(serial_robot_mdh_path, delimiter=',')
    kc = KinematicChain.from_array(array)
    return Robot(kc)


@fixture(name='planar_manipulator')
def kinematic_chain_fixture():
    """
    Generate planar robot model.

    :return:
    """
    planar_manipulator = np.array([
        [0, 0, 0, 0],
        [0, 10, 0, 0],
        [0, 20, 0, 0]
    ])
    kc = KinematicChain.from_array(planar_manipulator)
    return Robot(kc)
