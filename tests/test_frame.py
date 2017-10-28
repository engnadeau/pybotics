"""Test frame."""
import numpy as np
from pytest import raises

from pybotics.constants import TRANSFORM_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE
from pybotics.errors import Matrix4x4Error, SequenceLengthError, \
    OrientationConventionError


def test_optimization(world_frame):
    masked_index = 1

    # test mask
    world_frame.optimization_mask = True
    # noinspection PyTypeChecker
    assert all(world_frame.optimization_mask)

    mask = [True] * TRANSFORM_VECTOR_LENGTH
    mask[masked_index] = False
    world_frame.optimization_mask = mask
    assert sum(world_frame.optimization_mask) == TRANSFORM_VECTOR_LENGTH - 1

    # test optimization vector
    masked_element = world_frame.vector()[masked_index]
    new_optimization_vector = world_frame.optimization_vector * 2
    world_frame.apply_optimization_vector(new_optimization_vector)

    assert world_frame.vector()[masked_index] == masked_element

    leftover_vector = [e for i, e in enumerate(world_frame.vector()) if
                       i != masked_index]
    np.testing.assert_allclose(leftover_vector, new_optimization_vector)


def test_matrix(world_frame):
    new_matrix = np.ones(TRANSFORM_MATRIX_SHAPE)
    world_frame.matrix = new_matrix
    np.testing.assert_allclose(world_frame.matrix, new_matrix)

    bad_matrix = np.ones((4, 1))
    with raises(Matrix4x4Error):
        world_frame.matrix = bad_matrix


def test_position(world_frame):
    new_position = np.ones(3)
    world_frame.position = new_position
    np.testing.assert_allclose(world_frame.position, new_position)

    bad_position = np.ones(4)
    with raises(SequenceLengthError):
        world_frame.position = bad_position


def test_vector(world_frame):
    with raises(OrientationConventionError):
        world_frame.vector(123)
