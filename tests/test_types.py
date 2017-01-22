import numpy as np
from pybotics.types import Vector


def test_vector():
    x = [1.1, 2.2, 3.3]
    assert isinstance(x, Vector)

    x = np.array([1.1, 2.2, 3.3])
    assert isinstance(x, Vector)

    x = (1.1, 2.2, 3.3)
    assert isinstance(x, Vector)
    pass
