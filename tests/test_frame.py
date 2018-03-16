"""Test frame."""
import numpy as np

from pybotics.constants import TRANSFORM_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE


def test_optimization(world_frame):
    """
    Test frame.

    :param world_frame:
    :return:
    """
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
    """
    Test frame.

    :param world_frame:
    :return:
    """
    new_matrix = np.ones(TRANSFORM_MATRIX_SHAPE)
    world_frame.matrix = new_matrix
    np.testing.assert_allclose(world_frame.matrix, new_matrix)


def test_position(world_frame):
    """
    Test frame.

    :param world_frame:
    :return:
    """
    new_position = np.ones(3)
    world_frame.joints = new_position
    np.testing.assert_allclose(world_frame.joints, new_position)
