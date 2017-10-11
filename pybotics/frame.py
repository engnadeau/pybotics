from copy import deepcopy
from itertools import compress
from typing import Union, List

import numpy as np  # type: ignore
from pybotics.matrix import Matrix
from pybotics.validation import is_4x4_ndarray
from pybotics.vector import Vector
from pybotics.geometry import matrix_2_euler_zyx, euler_zyx_2_matrix
from pybotics.constant import Constant
from pybotics.optimizable import Optimizable


class Frame(Optimizable, Vector, Matrix):
    @property
    def optimization_mask(self) -> List[bool]:
        return self._optimization_mask

    @optimization_mask.setter
    def optimization_mask(self, value: Union[bool, List[bool]]) -> None:
        if isinstance(value, bool):
            self._optimization_mask = [value] * Constant.TRANSFORM_VECTOR_LENGTH.value
        else:
            self._optimization_mask = value

    @property
    def optimization_vector(self) -> np.ndarray:
        return np.array(list(compress(self.vector, self.optimization_mask)))

    @optimization_vector.setter
    def optimization_vector(self, value: np.ndarray) -> None:
        vector = deepcopy(self.vector)
        mask = deepcopy(self.optimization_mask)
        it = iter(value)
        self.vector = [v if not m else next(it) for v, m in zip(vector, mask)]

    def __init__(self, matrix: np.ndarray = None) -> None:
        super().__init__()
        self._matrix = None
        self.matrix = np.eye(4) if matrix is None else matrix

    @property
    def vector(self) -> np.ndarray:
        return matrix_2_euler_zyx(self.matrix)

    @vector.setter
    def vector(self, vector: np.ndarray) -> None:
        self.matrix = euler_zyx_2_matrix(vector)

    @property
    def matrix(self) -> np.ndarray:
        return self._matrix

    @matrix.setter
    def matrix(self, value: np.ndarray) -> None:
        if not is_4x4_ndarray(value):
            raise ValueError('Matrix must be 4x4 ndarray')
        else:
            self._matrix = value
