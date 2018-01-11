"""Test geometry."""
import hypothesis.strategies as st
import numpy as np
from hypothesis import given
from pytest import raises

from pybotics import geometry
from pybotics.constants import TRANSFORM_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE
from pybotics.errors import SequenceError, Matrix4x4Error
from pybotics.geometry import wrap_2_pi, euler_zyx_2_matrix, \
    matrix_2_euler_zyx, translation_matrix, rotation_matrix_x, \
    rotation_matrix_y, rotation_matrix_z

EULER_ZYX_VECTOR = np.array([100, 200, 300,
                             np.deg2rad(-30), np.deg2rad(50), np.deg2rad(90)])
TRANSFORM = np.array([
    [0, -0.642788, 0.766044, 100],
    [0.866025, 0.383022, 0.321394, 200],
    [-0.5, 0.663414, 0.556670, 300],
    [0, 0, 0, 1]
])


def test_euler_zyx_2_matrix():
    """
    Test conversion.

    :return:
    """
    actual = euler_zyx_2_matrix(EULER_ZYX_VECTOR)
    np.testing.assert_allclose(actual=actual, desired=TRANSFORM, atol=1e-6)

    with raises(SequenceError):
        euler_zyx_2_matrix(np.ones(TRANSFORM_VECTOR_LENGTH * 2))


def test_matrix_2_euler_zyx():
    """
    Test conversion.

    :return:
    """
    # test normal function
    actual = matrix_2_euler_zyx(TRANSFORM)
    np.testing.assert_allclose(actual=actual, desired=EULER_ZYX_VECTOR,
                               atol=1e-6)

    # test validation
    with raises(Matrix4x4Error):
        matrix_2_euler_zyx(np.ones(TRANSFORM_VECTOR_LENGTH))

    # test matrix decomposition corner cases when y=90deg
    corner_case_matrix = np.array(
        [0, 0, 1, 0,
         0, 1, 0, 0,
         -1, 0, 0, 0,
         0, 0, 0, 1]
    ).reshape(TRANSFORM_MATRIX_SHAPE)
    desired = [0, 0, 0, 0, np.deg2rad(90), 0]
    actual = matrix_2_euler_zyx(corner_case_matrix)
    np.testing.assert_allclose(actual=actual, desired=desired, atol=1e-6)

    # test matrix decomposition corner cases when y=-90deg
    corner_case_matrix = np.array(
        [0, 0, -1, 0,
         0, 1, 0, 0,
         1, 0, 0, 0,
         0, 0, 0, 1]
    ).reshape(TRANSFORM_MATRIX_SHAPE)
    desired = [0, 0, 0, 0, np.deg2rad(-90), 0]
    actual = matrix_2_euler_zyx(corner_case_matrix)
    np.testing.assert_allclose(actual=actual, desired=desired, atol=1e-6)


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_wrap_2_pi(angle):
    """
    Test angle wrapping.

    :return:
    """
    # hypothesis testing
    assert -np.pi <= wrap_2_pi(angle) < np.pi

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

    actual_angles = np.array(list(map(wrap_2_pi, test_angles)))
    assert len(test_angles) == len(expected_angles)
    assert len(actual_angles) == len(expected_angles)
    np.testing.assert_allclose(actual_angles, expected_angles)

    # test single elements
    for i, _ in enumerate(expected_angles):
        actual_angle = wrap_2_pi(test_angles[i])
        np.testing.assert_allclose([actual_angle], expected_angles[i])


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_rotation_matrix(angle):
    # iterate through rotation axes
    for i, axis in enumerate('xyz'):
        # getattr() could have been used
        # but it doesn't show that the function is `used`
        # the if-else structure avoids `dead code` errors
        if axis is 'x':
            matrix = rotation_matrix_x(angle)
        elif axis is 'y':
            matrix = rotation_matrix_y(angle)
        elif axis is 'z':
            matrix = rotation_matrix_z(angle)

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


@given(st.lists(st.floats(allow_nan=False, allow_infinity=False),
                min_size=3, max_size=3))
def test_translation_matrix(xyz):
    matrix = translation_matrix(xyz)

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
