"""Test geometry."""
from collections import Counter
from pathlib import Path
from typing import Sequence

import hypothesis.strategies as st
import numpy as np
from hypothesis import given, settings
from hypothesis.extra.numpy import arrays
from pytest import raises

import pybotics.geometry
from pybotics.errors import PyboticsError
from pybotics.geometry import OrientationConvention, matrix_2_vector


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_wrap_2_pi(angle):
    """
    Test angle wrapping.

    :return:
    """
    # hypothesis testing
    assert -np.pi <= pybotics.geometry.wrap_2_pi(angle) < np.pi

    # standard testing
    angles = np.array(
        [[0, 0], [-np.pi, -np.pi], [np.pi, -np.pi], [2 * np.pi, 0], [-2 * np.pi, 0]]
    )

    test_angles = angles[:, 0]
    expected_angles = angles[:, 1]

    actual_angles = np.array(list(map(pybotics.geometry.wrap_2_pi, test_angles)))
    assert len(test_angles) == len(expected_angles)
    assert len(actual_angles) == len(expected_angles)
    np.testing.assert_allclose(actual_angles, expected_angles)

    # test single elements
    for i, _ in enumerate(expected_angles):
        actual_angle = pybotics.geometry.wrap_2_pi(test_angles[i])
        np.testing.assert_allclose([actual_angle], expected_angles[i])


@given(angle=st.floats(allow_nan=False, allow_infinity=False))
@settings(deadline=None)
def test_rotation_matrix_xyz(angle, resources_path: Path):
    """Test."""
    # define functions to test
    rotation_functions = {
        "x": pybotics.geometry.rotation_matrix_x,
        "y": pybotics.geometry.rotation_matrix_y,
        "z": pybotics.geometry.rotation_matrix_z,
    }

    # iterate through rotation axes
    for i, axis in enumerate("xyz"):
        # get resource file
        path = resources_path / f"rot{axis}-transforms.csv"
        data = np.loadtxt(str(path.resolve()), delimiter=",")
        if data.ndim == 1:
            data = np.expand_dims(data, axis=0)

        # test resource transforms
        for d in data:
            actual_matrix = rotation_functions[axis](d[0])
            np.testing.assert_allclose(
                actual=actual_matrix, desired=d[1:].reshape((4, 4)), atol=1e-6
            )

        # test hypothesis transforms
        actual_matrix = rotation_functions[axis](angle)

        # check orthogonality
        for row in actual_matrix:
            # noinspection PyTypeChecker
            np.testing.assert_allclose(np.linalg.norm(row), 1)

        for column in actual_matrix.T:
            # noinspection PyTypeChecker
            np.testing.assert_allclose(np.linalg.norm(column), 1)

        # check no translation
        # noinspection PyTypeChecker
        np.testing.assert_allclose(actual_matrix[:-1, -1], 0)

        # check homogeneous matrix
        # noinspection PyTypeChecker
        np.testing.assert_allclose(actual_matrix[-1, :-1], 0)

        # check unit vector location
        # noinspection PyTypeChecker
        np.testing.assert_allclose(actual_matrix[i, i], 1)


@given(
    arrays(
        shape=(3,),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    )
)
def test_translation_matrix(xyz):
    """Test."""
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

    # test exception
    with raises(PyboticsError):
        pybotics.geometry.translation_matrix(np.zeros(10))


def test_vector_2_matrix(vector_transforms: Sequence[dict]):
    """Test."""
    # test regular usage
    for d in vector_transforms:
        for c in [d["order"], OrientationConvention(d["order"])]:
            actual = pybotics.geometry.vector_2_matrix(d["vector"], convention=c)
            np.testing.assert_allclose(
                actual=actual, desired=d["transform"].reshape((4, 4)), atol=1e-6
            )

        # test exception
        with raises(PyboticsError):
            pybotics.geometry.vector_2_matrix(d["vector"], convention="foobar")


def test_matrix_2_vector(vector_transforms: Sequence[dict]):
    """Test."""
    for d in vector_transforms:
        for c in [
            e
            for e in OrientationConvention.__members__.values()
            if d["order"] == e.value
        ]:
            try:
                actual_vector = matrix_2_vector(d["transform"].reshape((4, 4)), c)
            except NotImplementedError:
                # TODO: implement other conversions
                # don't fail for NotImplementedError
                continue
            np.testing.assert_allclose(
                actual=actual_vector, desired=d["vector"], atol=1e-6
            )


def test_orientation():
    """Test."""
    # ensure order and name match
    for e in list(OrientationConvention.__members__.values()):
        name_order = e.name.split("_")[-1].lower()
        assert name_order == e.value

    # ensure that there are only two of each value (euler and fixed)
    values = [e.value for e in OrientationConvention.__members__.values()]
    counts = Counter(values).values()
    assert all([v == 2 for v in counts])

    # ensure only x,y,z are used
    good_letters = set("xyz")
    values = set([e.value for e in OrientationConvention.__members__.values()])
    leftover_values = [x for x in values if set(x).difference(good_letters)]
    assert not leftover_values
