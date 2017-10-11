from typing import Optional, List
import numpy as np

from pybotics.link import Link


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


def is_same_link_conventions(links: List[Link]) -> bool:
    convention = None
    for link in links:
        if convention is None:
            convention = link.convention
        else:
            if link.convention is not convention:
                return False
    return True
