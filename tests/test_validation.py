import numpy as np

from pybotics.validation import is_1d_ndarray, is_4x4_ndarray


def test_is_1d_ndarray():
    # good, usual case
    x = np.eye(1, 3).ravel()
    assert is_1d_ndarray(x)
    assert is_1d_ndarray(x, 3)

    # bad, almost 1D array
    x = np.eye(1, 3)
    assert not is_1d_ndarray(x)

    # bad length
    x = np.eye(1, 3).ravel()
    assert not is_1d_ndarray(x, 4)

    # bad 2D array
    x = np.eye(4)
    assert not is_1d_ndarray(x)

    # bad list
    x = list(np.eye(1, 3))
    assert not is_1d_ndarray(x)


def test_is_4x4_ndarray():
    # good, usual case
    x = np.eye(4)
    assert is_4x4_ndarray(x)

    # bad array
    x = np.eye(1, 3)
    assert not is_4x4_ndarray(x)

    # bad list
    x = list(np.eye(4))
    assert not is_4x4_ndarray(x)
