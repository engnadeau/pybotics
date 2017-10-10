from typing import Optional
import numpy as np


def is_4x4_ndarray(value: np.ndarray) -> bool:
    result = isinstance(value, np.ndarray)

    if result:
        result = result and np.array_equal(value.shape, [4, 4])

    return result


def is_1d_ndarray(value: np.ndarray, length: Optional[int] = None) -> bool:
    result = isinstance(value, np.ndarray)

    if result:
        result = result and value.ndim == 1

        if length is not None:
            result = result and value.size == length

    return result
