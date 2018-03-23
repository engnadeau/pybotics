"""Test."""
import numpy as np

from pybotics.validation import is_4x4_matrix, is_vector


def test_is_4x4_matrix():
    """Test."""
    assert is_4x4_matrix(np.eye(4))
    assert not is_4x4_matrix(np.zeros(5))
    assert not is_4x4_matrix(np.eye(5))


def test_is_vector():
    """Test."""
    assert is_vector(np.zeros(4), 4)
    assert not is_vector(np.zeros(4), 5)
    assert not is_vector(np.eye(4), 4)
