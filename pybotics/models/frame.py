from copy import deepcopy
from itertools import compress

import numpy as np

from pybotics.models.matrix import Matrix
from pybotics.models.optimizable import Optimizable
from pybotics.models.vector import Vector
from pybotics.utilities.constant import Constant
from pybotics.utilities.geometry import matrix_2_euler_zyx, euler_zyx_2_matrix
from pybotics.utilities.validation import is_4x4_ndarray


class Frame(Optimizable, Vector, Matrix):
    @property
    def optimization_mask(self):
        return self._optimization_mask

    @optimization_mask.setter
    def optimization_mask(self, value):
        if isinstance(value, bool):
            self._optimization_mask = [value] * Constant.TRANSFORM_VECTOR_LENGTH.value
        else:
            # TODO: input validation
            self._optimization_mask = value

    @property
    def optimization_vector(self):
        return np.array(list(compress(self.vector, self.optimization_mask)))

    @optimization_vector.setter
    def optimization_vector(self, value):
        vector = deepcopy(self.vector)
        mask = deepcopy(self.optimization_mask)
        it = iter(value)
        self.vector = [v if not m else next(it) for v, m in zip(vector, mask)]

    def __init__(self, matrix: np.ndarray = None) -> None:
        super().__init__()
        self._matrix = None
        self.matrix = np.eye(4) if matrix is None else matrix

    @property
    def vector(self):
        return matrix_2_euler_zyx(self.matrix)

    @vector.setter
    def vector(self, vector):
        self.matrix = euler_zyx_2_matrix(vector)

    @property
    def matrix(self):
        return self._matrix

    @matrix.setter
    def matrix(self, value):
        if not is_4x4_ndarray(value):
            raise ValueError('Matrix must be 4x4 ndarray')
        else:
            self._matrix = value
