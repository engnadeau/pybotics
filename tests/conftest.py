"""Pytest config."""
import os

import numpy as np
from pytest import fixture

from pybotics.frame import Frame
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot
from pybotics.tool import Tool


@fixture(name='serial_robot')
def serial_robot_fixture(serial_kc):
    """
    Generate serial robot.

    :return:
    """
    return Robot(serial_kc)


@fixture(name='serial_kc')
def serial_kc_fixture():
    """
    Generate serial kinematic chain.

    :return:
    """
    ur10_mdh_path = os.path.abspath(os.path.join(
        os.path.dirname(__file__),
        os.pardir,
        'robot-models',
        'ur10-mdh.csv'))

    mdh = np.loadtxt(ur10_mdh_path, delimiter=',')
    kc = KinematicChain.from_array(mdh)
    return kc


@fixture(name='planar_robot')
def planar_robot_fixture(planar_kc):
    """
    Generate planar robot.

    :return:
    """
    return Robot(planar_kc)


@fixture(name='planar_kc')
def planar_kc_fixture():
    """
    Generate planar kinematic chain.

    :return:
    """
    mdh = np.array([
        [0, 0, 0, 0],
        [0, 10, 0, 0],
        [0, 20, 0, 0]
    ])
    kc = KinematicChain.from_array(mdh)
    return kc


@fixture(name='tool')
def tool_fixture():
    """
    Generate tool.

    :return:
    """
    tool = Tool()
    tool.position = [10, 20, 30]
    tool.mass = 5
    tool.cg = [-10, -20, -30]
    return tool


@fixture(name='world_frame')
def world_frame_fixture():
    """
    Generate tool.

    :return:
    """
    frame = Frame()
    frame.position = [10, 20, 30]
    return frame
