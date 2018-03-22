from typing import Sequence

import numpy as np  # type: ignore

from pybotics.constants import TRANSFORM_MATRIX_SHAPE


def is_4x4_matrix(x: np.ndarray) -> bool:
    if not x.ndim == 2:
        return False
    if not x.shape == TRANSFORM_MATRIX_SHAPE:
        return False
    return True


def is_vector(x: Sequence, required_length: int) -> bool:
    if not len(x) == required_length:
        return False
    if isinstance(x, np.ndarray):
        if x.ndim != 1:
            return False
    return True
