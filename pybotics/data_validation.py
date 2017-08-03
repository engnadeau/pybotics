from typing import Optional

import numpy as np


def validate_4x4_matrix(value: np.ndarray):
    if not np.array_equal(value.shape, [4, 4]):
        raise ValueError('Required: 4x4 matrix.')


def validate_1d_vector(value: np.ndarray, length: Optional[int] = None):
    if value.ndim != 1:
        raise ValueError('Required: 1D ndarray.')

    if length is not None:
        if value.size != length:
            raise ValueError('Required: ndarray of length {}.'.format(length))
