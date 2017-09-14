import numpy as np

from pybotics.utilities.geometry import matrix_2_euler_zyx, euler_zyx_2_matrix
from pybotics.utilities.validation import is_4x4_ndarray


class Frame:
    def __init__(self, matrix: np.ndarray = None) -> None:
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
