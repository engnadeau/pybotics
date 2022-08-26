"""Test geometry."""
import typing
from collections import Counter
from pathlib import Path
from typing import Dict, List, Sequence, Union

import hypothesis.strategies as st
import numpy as np
import numpy.typing as npt
from hypothesis import given
from hypothesis.extra.numpy import arrays
from pytest import fixture, raises
from scipy.spatial.transform import Rotation  # type: ignore

import pybotics.geometry
from pybotics.errors import PyboticsError
from pybotics.geometry import OrientationConvention, matrix_2_vector

MIN_FLOAT = -1e9
MAX_FLOAT = 1e9


@fixture()
def vector_transforms() -> List[Dict[str, Union[npt.NDArray[np.float64], str]]]:
    """Get resource data."""

    # load test data
    data_path = (
        Path(__file__).parent / "resources"
    ).resolve() / "vector-transforms.csv"
    data = np.genfromtxt(fname=data_path, delimiter=",", dtype=str)  # type: ignore

    result = [
        {
            "vector": d[:6].astype(float),
            "transform": d[6:-1].astype(float),
            "order": d[-1],
        }
        for d in data
    ]
    return result


@given(st.floats(allow_nan=False, allow_infinity=False))
def test_wrap_2_pi(angle: float) -> None:
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
    np.testing.assert_allclose(actual_angles, expected_angles)  # type: ignore

    # test single elements
    for i, _ in enumerate(expected_angles):
        actual_angle = pybotics.geometry.wrap_2_pi(test_angles[i])
        np.testing.assert_allclose([actual_angle], expected_angles[i])  # type: ignore


@given(
    angle=st.floats(
        allow_nan=False, allow_infinity=False, min_value=MIN_FLOAT, max_value=MAX_FLOAT
    )
)
def test_rotation_matrix_x(angle: float) -> None:
    # set axis-specific parameters
    axis_idx = 0
    axis_vector = np.zeros(3)
    axis_vector[axis_idx] = 1

    # test randomized hypothesis transforms
    actual_matrix = pybotics.geometry.rotation_matrix_x(angle)

    # check orthogonality
    for row in actual_matrix:
        np.testing.assert_allclose(np.linalg.norm(row), 1)  # type: ignore

    for column in actual_matrix.T:
        np.testing.assert_allclose(np.linalg.norm(column), 1)  # type: ignore

    # check no translation
    np.testing.assert_allclose(actual_matrix[:-1, -1], 0)  # type: ignore

    # check homogeneous matrix
    np.testing.assert_allclose(actual_matrix[-1, :-1], 0)  # type: ignore

    # check unit vector location
    np.testing.assert_allclose(actual_matrix[axis_idx, axis_idx], 1)  # type: ignore

    # check 3x3 rotation component
    actual_rotation = actual_matrix[:3, :3]
    expected_rotation = Rotation.from_rotvec(angle * axis_vector).as_matrix()
    np.testing.assert_allclose(  # type: ignore
        actual=actual_rotation, desired=expected_rotation, rtol=1e-05, atol=1e-08
    )


@given(
    angle=st.floats(
        allow_nan=False, allow_infinity=False, min_value=MIN_FLOAT, max_value=MAX_FLOAT
    )
)
def test_rotation_matrix_y(angle: float) -> None:
    # set axis-specific parameters
    axis_idx = 1
    axis_vector = np.zeros(3)
    axis_vector[axis_idx] = 1

    # test randomized hypothesis transforms
    actual_matrix = pybotics.geometry.rotation_matrix_y(angle)

    # check orthogonality
    for row in actual_matrix:
        np.testing.assert_allclose(np.linalg.norm(row), 1)  # type: ignore

    for column in actual_matrix.T:
        np.testing.assert_allclose(np.linalg.norm(column), 1)  # type: ignore

    # check no translation
    np.testing.assert_allclose(actual_matrix[:-1, -1], 0)  # type: ignore

    # check homogeneous matrix
    np.testing.assert_allclose(actual_matrix[-1, :-1], 0)  # type: ignore

    # check unit vector location
    np.testing.assert_allclose(actual_matrix[axis_idx, axis_idx], 1)  # type: ignore

    # check 3x3 rotation component
    actual_rotation = actual_matrix[:3, :3]
    expected_rotation = Rotation.from_rotvec(angle * axis_vector).as_matrix()
    np.testing.assert_allclose(  # type: ignore
        actual=actual_rotation, desired=expected_rotation, rtol=1e-05, atol=1e-08
    )


@given(
    angle=st.floats(
        allow_nan=False, allow_infinity=False, min_value=MIN_FLOAT, max_value=MAX_FLOAT
    )
)
def test_rotation_matrix_z(angle: float) -> None:
    # set axis-specific parameters
    axis_idx = 2
    axis_vector = np.zeros(3)
    axis_vector[axis_idx] = 1

    # test randomized hypothesis transforms
    actual_matrix = pybotics.geometry.rotation_matrix_z(angle)

    # check orthogonality
    for row in actual_matrix:
        np.testing.assert_allclose(np.linalg.norm(row), 1)  # type: ignore

    for column in actual_matrix.T:
        np.testing.assert_allclose(np.linalg.norm(column), 1)  # type: ignore

    # check no translation
    np.testing.assert_allclose(actual_matrix[:-1, -1], 0)  # type: ignore

    # check homogeneous matrix
    np.testing.assert_allclose(actual_matrix[-1, :-1], 0)  # type: ignore

    # check unit vector location
    np.testing.assert_allclose(actual_matrix[axis_idx, axis_idx], 1)  # type: ignore

    # check 3x3 rotation component
    actual_rotation = actual_matrix[:3, :3]
    expected_rotation = Rotation.from_rotvec(angle * axis_vector).as_matrix()
    np.testing.assert_allclose(  # type: ignore
        actual=actual_rotation, desired=expected_rotation, rtol=1e-05, atol=1e-08
    )


@given(
    arrays(
        shape=(3,),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    )
)
def test_translation_matrix(xyz: npt.NDArray[np.float64]) -> None:
    """Test."""
    matrix = pybotics.geometry.translation_matrix(xyz)

    # check orthogonality
    for row in matrix[:-1, :-1]:
        # noinspection PyTypeChecker
        np.testing.assert_allclose(np.linalg.norm(row), 1)  # type: ignore

    for column in matrix[:, :-1].T:
        # noinspection PyTypeChecker
        np.testing.assert_allclose(np.linalg.norm(column), 1)  # type: ignore

    # check translation
    # noinspection PyTypeChecker
    np.testing.assert_allclose(matrix[:-1, -1], xyz)  # type: ignore

    # check homogeneous matrix
    # noinspection PyTypeChecker
    np.testing.assert_allclose(matrix[-1, :-1], 0)  # type: ignore

    # test exception
    with raises(PyboticsError):
        pybotics.geometry.translation_matrix(np.zeros(10))


def test_vector_2_matrix(
    vector_transforms: Sequence[typing.Dict[str, npt.NDArray[np.float64]]]
) -> None:
    """Test."""
    # test regular usage
    for d in vector_transforms:
        for c in [d["order"], OrientationConvention(d["order"])]:
            actual = pybotics.geometry.vector_2_matrix(
                d["vector"], convention=c  # type: ignore
            )
            np.testing.assert_allclose(  # type: ignore
                actual=actual, desired=d["transform"].reshape((4, 4)), atol=1e-6
            )

        # test exception
        with raises(PyboticsError):
            pybotics.geometry.vector_2_matrix(d["vector"], convention="foobar")


def test_matrix_2_vector(
    vector_transforms: Sequence[typing.Dict[str, npt.NDArray[np.float64]]]
) -> None:
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
            np.testing.assert_allclose(  # type: ignore
                actual=actual_vector, desired=d["vector"], atol=1e-6
            )


def test_orientation() -> None:
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
    values = list(set([e.value for e in OrientationConvention.__members__.values()]))
    leftover_values = [x for x in values if set(x).difference(good_letters)]
    assert not leftover_values
