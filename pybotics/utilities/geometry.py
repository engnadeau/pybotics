"""Geometry functions and utilities."""
import math
from typing import Union

import numpy as np  # type: ignore

from pybotics.utilities.validation import is_4x4_ndarray


def euler_zyx_2_matrix(vector: np.ndarray) -> np.ndarray:
    """
    Calculate the pose from the position and euler angles.

    :param vector:
    :return:
    """
    # get individual variables
    [x, y, z, a, b, c] = vector

    # get trig values
    ca = np.cos(a)
    sa = np.sin(a)

    cb = np.cos(b)
    sb = np.sin(b)

    cc = np.cos(c)
    sc = np.sin(c)

    # get resulting transform
    transform = [
        [cb * cc, -cb * sc, sb, x],
        [ca * sc + cc * sa * sb, ca * cc - sa * sb * sc, -cb * sa, y],
        [sa * sc - ca * cc * sb, cc * sa + ca * sb * sc, ca * cb, z],
        [0, 0, 0, 1]
    ]

    return np.array(transform, dtype=np.float)


def matrix_2_euler_zyx(matrix: np.ndarray) -> np.ndarray:
    """
    Calculate the equivalent position and euler angles of the given pose.

    From: Craig, John J. Introduction to robotics: mechanics and control, 2005
    """
    if not is_4x4_ndarray(matrix):
        raise ValueError('4x4 transformation matrix is required')

    x = matrix[0, 3]
    y = matrix[1, 3]
    z = matrix[2, 3]

    # solution degenerates near ry = +/- 90deg
    cos_ry = np.sqrt(matrix[1, 2] ** 2 + matrix[2, 2] ** 2)
    sin_ry = matrix[0, 2]

    if np.isclose([cos_ry], [0]):
        rx = 0.0
        if sin_ry > 0:
            ry = np.pi / 2
            rz = np.arctan2(matrix[1, 0], -matrix[2, 0])
        else:
            ry = -math.pi / 2
            rz = math.atan2(matrix[1, 0], matrix[2, 0])
    else:
        sin_rx = -matrix[1, 2] / cos_ry
        cos_rx = matrix[2, 2] / cos_ry
        rx = math.atan2(sin_rx, cos_rx)

        ry = math.atan2(sin_ry, cos_ry)

        sin_rz = -matrix[0, 1] / cos_ry
        cos_rz = matrix[0, 0] / cos_ry
        rz = math.atan2(sin_rz, cos_rz)

    return np.array([x, y, z, rx, ry, rz])


def wrap_2_pi(angles: Union[np.ndarray, float]) -> Union[np.ndarray, float]:
    """
    Recursively wrap given angles to +/- PI.

    :param angles:
    :return:
    """
    return (angles + np.pi) % (2 * np.pi) - np.pi
