"""Geometry functions and utilities."""
from typing import Sequence

import numpy as np  # type: ignore

from pybotics.constants import TRANSFORM_VECTOR_LENGTH
from pybotics.errors import Matrix4x4Error, SequenceError
from pybotics.validation import is_4x4_ndarray, is_1d_sequence


def euler_zyx_2_matrix(vector: Sequence[float]) -> np.ndarray:
    """
    Calculate the pose from the position and euler angles.

    :param vector: transform vector
    :return: 4x4 transform matrix
    """
    # validate input
    if not is_1d_sequence(vector, TRANSFORM_VECTOR_LENGTH):
        raise SequenceError('vector', TRANSFORM_VECTOR_LENGTH)

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
    :param matrix: 4x4 transform matrix
    :return: transform vector
    """
    if not is_4x4_ndarray(matrix):
        raise Matrix4x4Error('matrix')

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
            ry = -np.pi / 2
            rz = np.arctan2(matrix[1, 0], matrix[2, 0])
    else:
        sin_rx = -matrix[1, 2] / cos_ry
        cos_rx = matrix[2, 2] / cos_ry
        rx = np.arctan2(sin_rx, cos_rx)

        ry = np.arctan2(sin_ry, cos_ry)

        sin_rz = -matrix[0, 1] / cos_ry
        cos_rz = matrix[0, 0] / cos_ry
        rz = np.arctan2(sin_rz, cos_rz)

    return np.array([x, y, z, rx, ry, rz])


def wrap_2_pi(angle: float) -> float:
    """
    Wrap given angle to +/- PI.

    :param angle: angle to wrap
    :return: wrapped angle
    """
    # FIXME: remove float() cast when numpy is supported in mypy
    result = float((angle + np.pi) % (2 * np.pi) - np.pi)
    return result
