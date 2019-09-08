"""Geometry functions and utilities."""
from enum import Enum
from typing import Sequence, Union

import numpy as np  # type: ignore

from pybotics.errors import PyboticsError


class OrientationConvention(Enum):
    """Orientation of a body with respect to a fixed coordinate system."""

    EULER_XYX = "xyx"
    EULER_XYZ = "xyz"
    EULER_XZX = "xzx"
    EULER_XZY = "xzy"
    EULER_YXY = "yxy"
    EULER_YXZ = "yxz"
    EULER_YZX = "yzx"
    EULER_YZY = "yzy"
    EULER_ZXY = "zxy"
    EULER_ZXZ = "zxz"
    EULER_ZYX = "zyx"
    EULER_ZYZ = "zyz"

    FIXED_XYX = "xyx"
    FIXED_XYZ = "zyx"
    FIXED_XZX = "xzx"
    FIXED_XZY = "yzx"
    FIXED_YXY = "yxy"
    FIXED_YXZ = "zxy"
    FIXED_YZX = "xzy"
    FIXED_YZY = "yzy"
    FIXED_ZXY = "yxz"
    FIXED_ZXZ = "zxz"
    FIXED_ZYX = "xyz"
    FIXED_ZYZ = "zyz"


def vector_2_matrix(
    vector: Sequence[float],
    convention: Union[OrientationConvention, str] = OrientationConvention.EULER_ZYX,
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
    if isinstance(convention, OrientationConvention):
        convention = convention.value
    try:
        OrientationConvention(convention)
    except ValueError as e:
        raise PyboticsError(str(e))

    # iterate through rotation order
    # build rotation matrix
    transform_matrix = np.eye(4)
    for axis, value in zip(convention, rotation_component):  # type: ignore
        current_rotation = globals()[f"rotation_matrix_{axis}"](value)
        transform_matrix = np.dot(transform_matrix, current_rotation)

    # add translation component
    transform_matrix[:-1, -1] = translation_component

    return transform_matrix


def position_from_matrix(matrix: np.ndarray) -> np.ndarray:
    """Get the position values from a 4x4 transform matrix."""
    return matrix[:-1, -1]


def matrix_2_vector(
    matrix: np.ndarray,
    convention: OrientationConvention = OrientationConvention.EULER_ZYX,
) -> np.ndarray:
    """Convert 4x4 matrix to a vector."""
    # call function
    try:
        return globals()[f"_matrix_2_{convention.name.lower()}"](matrix)
    except KeyError:  # pragma: no cover
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

    vector = np.hstack((matrix[:-1, -1], [a, b, c]))
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

    matrix = np.array([1, 0, 0, 0, 0, c, -s, 0, 0, s, c, 0, 0, 0, 0, 1]).reshape((4, 4))

    return matrix


def rotation_matrix_y(angle: float) -> np.ndarray:
    """Generate a basic 4x4 rotation matrix about the Y axis."""
    s = np.sin(angle)
    c = np.cos(angle)

    matrix = np.array([c, 0, s, 0, 0, 1, 0, 0, -s, 0, c, 0, 0, 0, 0, 1]).reshape((4, 4))

    return matrix


def rotation_matrix_z(angle: float) -> np.ndarray:
    """Generate a basic 4x4 rotation matrix about the Z axis."""
    s = np.sin(angle)
    c = np.cos(angle)

    matrix = np.array([c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]).reshape((4, 4))

    return matrix


def translation_matrix(xyz: Sequence[float]) -> np.ndarray:
    """Generate a basic 4x4 translation matrix."""
    # validate
    if len(xyz) != 3:
        raise PyboticsError("len(xyz) must be 3")

    matrix = np.eye(4)
    matrix[:-1, -1] = xyz

    return matrix
