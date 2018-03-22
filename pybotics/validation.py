"""Validation helpers."""
from typing import Sequence

import numpy as np  # type: ignore

from pybotics.constants import TRANSFORM_MATRIX_SHAPE


def is_4x4_matrix(x: np.ndarray) -> bool:
    """Check if 4x4 ndarray."""
    if not x.ndim == 2:
        return False
    if not x.shape == TRANSFORM_MATRIX_SHAPE:
        return False
    return True


def is_vector(x: Sequence, required_length: int) -> bool:
    """Check if 1D vector."""
    if not len(x) == required_length:
        return False
    if np.ndim(x) != 1:
        return False
    return True
