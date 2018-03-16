"""Pytest config."""
from typing import Tuple

import numpy as np
from pytest import fixture

from pybotics.constants import TRANSFORM_MATRIX_SHAPE
from pybotics.frame import Frame
from pybotics.kinematic_chain import KinematicChain
from pybotics.robot import Robot
from pybotics.robot_models import UR10
from pybotics.tool import Tool


@fixture()
def planar_robot_link_lengths() -> Tuple[float, float]:
    return 10, 20


@fixture()
def serial_robot():
    """
    Generate serial robot.

    :return:
    """
    return UR10()


@fixture()
def planar_robot(planar_kc):
    """
    Generate planar robot.

    :return:
    """
    return Robot(planar_kc)


@fixture()
def planar_kc(planar_robot_link_lengths):
    """
    Generate planar kinematic chain.

    :return:
    """
    mdh = np.array([
        [0, 0, 0, 0],
        [0, planar_robot_link_lengths[0], 0, 0],
        [0, planar_robot_link_lengths[1], 0, 0]
    ])
    kc = KinematicChain.from_array(mdh)
    return kc


@fixture()
def tool():
    """
    Generate tool.

    :return:
    """
    tool = Tool()
    tool.position = [10, 20, 30]
    tool.mass = 5
    tool.cg = [-10, -20, -30]
    return tool


@fixture()
def world_frame():
    """
    Generate tool.

    :return:
    """
    frame = Frame()
    frame.position = [10, 20, 30]
    return frame


@fixture()
def vector_transform():
    euler_zyx_vector = np.array([100, 200, 300,
                                 np.deg2rad(10),
                                 np.deg2rad(20),
                                 np.deg2rad(30)])
    transform = np.array([
        0.925417, 0.018028, 0.378522, 100,
        0.163176, 0.882564, -0.440970, 200,
        -0.342020, 0.469846, 0.813798, 300,
        0.000000, 0.000000, 0.000000, 1.000000
    ]).reshape(TRANSFORM_MATRIX_SHAPE)

    return euler_zyx_vector, transform


@fixture()
def x_rotation_matrix():
    angle = np.deg2rad(10)
    transform = np.array([
        1.0000000, 0.0000000, 0.0000000, 0,
        0.0000000, 0.9848077, -0.1736482, 0,
        0.0000000, 0.1736482, 0.9848077, 0,
        0, 0, 0, 1
    ]).reshape(TRANSFORM_MATRIX_SHAPE)
    return angle, transform


@fixture()
def y_rotation_matrix():
    angle = np.deg2rad(10)
    transform = np.array([
        0.9848077, 0.0000000, 0.1736482, 0,
        0.0000000, 1.0000000, 0.0000000, 0,
        -0.1736482, 0.0000000, 0.9848077, 0,
        0, 0, 0, 1
    ]).reshape(TRANSFORM_MATRIX_SHAPE)
    return angle, transform


@fixture()
def z_rotation_matrix():
    angle = np.deg2rad(10)
    transform = np.array([
        0.9848077, -0.1736482, 0.0000000, 0,
        0.1736482, 0.9848077, 0.0000000, 0,
        0.0000000, 0.0000000, 1.0000000, 0,
        0, 0, 0, 1
    ]).reshape(TRANSFORM_MATRIX_SHAPE)
    return angle, transform
