import numpy as np
from typing import List

from pybotics.types import Vector


def test_vector():
    x = [1, 2, 3]
    assert isinstance(x, Vector.__union_params__)

    x = np.array([1, 2, 3])
    assert isinstance(x, Vector.__union_params__)
