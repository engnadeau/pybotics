"""Validation module."""
from typing import Optional, Sequence

import numpy as np  # type: ignore

from pybotics.link import Link


def is_1d_ndarray(array: np.ndarray, length: Optional[int] = None) -> bool:
    """
    Check if value is a 1D ndarray.

    :param array: value to check
    :param length: optional length to check
    :return: validation result
    """
    result = is_ndarray(array) and array.ndim == 1

    if length is not None:
        result = result and array.size == length

    return result


def is_4x4_ndarray(array: np.ndarray) -> bool:
    """
    Check if value is a 4x4 ndarray.

    :param array: value to check
    :return: validation result
    """
    return is_ndarray(array) and np.array_equal(array.shape, [4, 4])


def is_sequence_length_correct(sequence: Sequence, length: int) -> bool:
    """
    Check if sequence is the correct length.

    :param sequence: value to check
    :param length: length to check
    :return: validation result
    """
    return len(sequence) == length


def is_ndarray(array: np.ndarray) -> bool:
    """
    Check if value is a ndarray.

    :param array: value to check
    :return: validation result
    """
    return isinstance(array, np.ndarray)


def is_same_link_conventions(links: Sequence[Link]) -> bool:
    """
    Check if a sequence of links all use the same convention.

    :param links: sequence to check
    :return: validation result
    """
    convention = None
    for link in links:
        if convention is None:
            convention = link.convention
        else:
            if link.convention is not convention:
                return False
    return True
