"""Test geometry."""
import hypothesis.strategies as st
import numpy as np
from hypothesis import given
from hypothesis.extra.numpy import arrays

import pybotics.geometry
from pybotics.constants import POSITION_VECTOR_LENGTH, \
    TRANSFORM_VECTOR_LENGTH
from pybotics.conventions import Orientation
from pybotics.geometry import matrix_2_vector, vector_2_matrix, \
    _matrix_2_euler_zyx, rotation_matrix_y


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_wrap_2_pi(angle):
    """
    Test angle wrapping.

    :return:
    """
    # hypothesis testing
    assert -np.pi <= pybotics.geometry.wrap_2_pi(angle) < np.pi

    # standard testing
    angles = np.array([
        [0, 0],
        [-np.pi, -np.pi],
        [np.pi, -np.pi],
        [2 * np.pi, 0],
        [-2 * np.pi, 0]
    ])

    test_angles = angles[:, 0]
    expected_angles = angles[:, 1]

    actual_angles = np.array(
        list(map(pybotics.geometry.wrap_2_pi, test_angles)))
    assert len(test_angles) == len(expected_angles)
    assert len(actual_angles) == len(expected_angles)
    np.testing.assert_allclose(actual_angles, expected_angles)

    # test single elements
    for i, _ in enumerate(expected_angles):
        actual_angle = pybotics.geometry.wrap_2_pi(test_angles[i])
        np.testing.assert_allclose([actual_angle], expected_angles[i])


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_rotation_matrix(angle):
    # iterate through rotation axes
    for i, axis in enumerate('xyz'):
        # getattr() could have been used
        # but it doesn't show that the function is `used`
        # the if-else structure avoids `dead code` errors
        if axis is 'x':
            matrix = pybotics.geometry.rotation_matrix_x(angle)
        elif axis is 'y':
            matrix = pybotics.geometry.rotation_matrix_y(angle)
        elif axis is 'z':
            matrix = pybotics.geometry.rotation_matrix_z(angle)

        # check orthogonality
        for row in matrix:
            # noinspection PyTypeChecker
            np.testing.assert_allclose(np.linalg.norm(row), 1)

        for column in matrix.T:
            # noinspection PyTypeChecker
            np.testing.assert_allclose(np.linalg.norm(column), 1)

        # check no translation
        # noinspection PyTypeChecker
        np.testing.assert_allclose(matrix[:-1, -1], 0)

        # check homogeneous matrix
        # noinspection PyTypeChecker
        np.testing.assert_allclose(matrix[-1, :-1], 0)

        # check unit vector location
        # noinspection PyTypeChecker
        np.testing.assert_allclose(matrix[i, i], 1)


@given(arrays(shape=(POSITION_VECTOR_LENGTH,), dtype=float))
def test_translation_matrix(xyz):
    matrix = pybotics.geometry.translation_matrix(xyz)

    # check orthogonality
    for row in matrix[:-1, :-1]:
        # noinspection PyTypeChecker
        np.testing.assert_allclose(np.linalg.norm(row), 1)

    for column in matrix[:, :-1].T:
        # noinspection PyTypeChecker
        np.testing.assert_allclose(np.linalg.norm(column), 1)

    # check translation
    # noinspection PyTypeChecker
    np.testing.assert_allclose(matrix[:-1, -1], xyz)

    # check homogeneous matrix
    # noinspection PyTypeChecker
    np.testing.assert_allclose(matrix[-1, :-1], 0)


def test_vector_2_matrix(vector_transform: tuple):
    for c in [Orientation.EULER_ZYX, 'zyx']:
        actual = pybotics.geometry.vector_2_matrix(vector_transform[0],
                                                   convention=c)
        np.testing.assert_allclose(actual=actual,
                                   desired=vector_transform[1],
                                   atol=1e-6)


def test_rotation_matrix_x(x_rotation_matrix):
    actual = pybotics.geometry.rotation_matrix_x(x_rotation_matrix[0])
    np.testing.assert_allclose(actual=actual, desired=x_rotation_matrix[1],
                               atol=1e-6)


def test_rotation_matrix_y(y_rotation_matrix):
    actual = pybotics.geometry.rotation_matrix_y(y_rotation_matrix[0])
    np.testing.assert_allclose(actual=actual, desired=y_rotation_matrix[1],
                               atol=1e-6)


def test_rotation_matrix_z(z_rotation_matrix):
    actual = pybotics.geometry.rotation_matrix_z(z_rotation_matrix[0])
    np.testing.assert_allclose(actual=actual, desired=z_rotation_matrix[1],
                               atol=1e-6)


@given(arrays(shape=(TRANSFORM_VECTOR_LENGTH,), dtype=float))
def test_matrix_2_vector(vector):
    # hypothesis can't generate homogenous matrices,
    # so start with vector and convert
    matrix = vector_2_matrix(vector)

    for convention in Orientation:
        try:
            actual_vector = matrix_2_vector(matrix, convention)
        except NotImplementedError:
            # TODO: implement other conversions
            # don't fail for NotImplementedError
            continue
        actual_matrix = vector_2_matrix(actual_vector, convention)
        np.testing.assert_allclose(actual=actual_matrix, desired=matrix,
                                   atol=1e-6)


def test_matrix_2_euler_zyx(vector_transform):
    """
    Test conversion.

    :return:
    """
    # test normal function
    actual = _matrix_2_euler_zyx(vector_transform[1])
    np.testing.assert_allclose(actual=actual, desired=vector_transform[0],
                               atol=1e-6)

    # test matrix decomposition corner cases when y=90deg
    angle = np.deg2rad(90)
    matrix = rotation_matrix_y(angle)
    desired = [0, 0, 0, 0, angle, 0]
    actual = _matrix_2_euler_zyx(matrix)
    np.testing.assert_allclose(actual=actual, desired=desired, atol=1e-6)

    # test matrix decomposition corner cases when y=-90deg
    angle = np.deg2rad(-90)
    matrix = rotation_matrix_y(angle)
    desired = [0, 0, 0, 0, angle, 0]
    actual = _matrix_2_euler_zyx(matrix)
    np.testing.assert_allclose(actual=actual, desired=desired, atol=1e-6)
