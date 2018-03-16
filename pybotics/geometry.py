"""Geometry functions and utilities."""
from typing import Sequence, Union

import numpy as np  # type: ignore

from pybotics.constants import POSITION_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE
from pybotics.conventions import Orientation
from pybotics.errors import PyboticsError


def vector_2_matrix(
        vector: Sequence[float],
        convention: Union[Orientation, str] = Orientation.EULER_ZYX
) -> np.ndarray:
    """
        Calculate the pose from the position and euler angles.

        :param convention:
        :param vector: transform vector
        :return: 4x4 transform matrix
        """
    # get individual variables
    translation_component = vector[:3]
    rotation_component = vector[-3:]

    # validate and extract orientation info
    if isinstance(convention, Orientation):
        convention = convention.value
    elif convention is str and convention not in [e.value for e in Orientation]:
        raise PyboticsError(
            'Bad convention: see pybotics.conventions.Orientation '
            'for proper conventions.')

    # iterate through rotation order
    # build rotation matrix
    transform_matrix = np.eye(TRANSFORM_MATRIX_SHAPE[0])
    for axis, value in zip(convention, rotation_component):
        current_rotation = globals()['rotation_matrix_{}'.format(axis)](value)
        transform_matrix = np.dot(transform_matrix, current_rotation)

    # add translation component
    transform_matrix[:-1, -1] = translation_component

    return transform_matrix


def matrix_2_vector(
        matrix: np.ndarray,
        convention: Orientation = Orientation.EULER_ZYX) -> np.ndarray:
    # call function
    try:
        return \
            globals()['_matrix_2_{}'.format(convention.name.lower())](matrix)
    except KeyError:
        raise NotImplementedError


def _matrix_2_euler_zyx(matrix: np.ndarray) -> np.ndarray:
    """
    Calculate the equivalent position and euler angles of the given pose.

    From: Craig, John J. Introduction to robotics: mechanics and control, 2005
    :param matrix: 4x4 transform matrix
    :return: transform vector
    """
    # solution degenerates near ry = +/- 90deg
    sb = -matrix[2, 0]
    cb = np.sqrt(matrix[0, 0] ** 2 + matrix[1, 0] ** 2)

    if np.isclose(cb, 0):
        a = 0.0
        b = np.sign(sb) * np.pi / 2

        sc = matrix[0, 1]
        cc = matrix[1, 1]
        c = np.sign(sb) * np.arctan2(sc, cc)
    else:
        b = np.arctan2(sb, cb)

        sa = matrix[1, 0] / cb
        ca = matrix[0, 0] / cb
        a = np.arctan2(sa, ca)

        sc = matrix[2, 1] / cb
        cc = matrix[2, 2] / cb
        c = np.arctan2(sc, cc)

    vector = np.hstack((matrix[: -1, -1], [a, b, c]))
    return vector


def wrap_2_pi(angle: float) -> float:
    """
    Wrap given angle to +/- PI.

    :param angle: angle to wrap
    :return: wrapped angle
    """
    # FIXME: remove float() cast when numpy is supported in mypy
    result = float((angle + np.pi) % (2 * np.pi) - np.pi)
    return result


def rotation_matrix_x(angle: float) -> np.ndarray:
    """Generate a basic 4x4 rotation matrix about the X axis."""
    s = np.sin(angle)
    c = np.cos(angle)

    matrix = np.array([
        1, 0, 0, 0,
        0, c, -s, 0,
        0, s, c, 0,
        0, 0, 0, 1
    ]).reshape((4, 4))

    return matrix


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Generate a basic 4x4 rotation matrix about the Y axis."""
    s = np.sin(angle)
    c = np.cos(angle)

    matrix = np.array([
        c, 0, s, 0,
        0, 1, 0, 0,
        -s, 0, c, 0,
        0, 0, 0, 1
    ]).reshape((4, 4))

    return matrix


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Generate a basic 4x4 rotation matrix about the Z axis."""
    s = np.sin(angle)
    c = np.cos(angle)

    matrix = np.array([
        c, -s, 0, 0,
        s, c, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1
    ]).reshape((4, 4))

    return matrix


def translation_matrix(xyz: Sequence[float]) -> np.ndarray:
    """Generate a basic 4x4 translation matrix."""
    # validate
    if len(xyz) != POSITION_VECTOR_LENGTH:
        raise PyboticsError(
            'len(xyz) must be {}'.format(POSITION_VECTOR_LENGTH))

    matrix = np.eye(4)
    matrix[:-1, -1] = xyz

    return matrix
