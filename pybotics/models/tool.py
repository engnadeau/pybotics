"""Tool module."""
from typing import Union

import numpy as np

from pybotics.models.frame import Frame
from pybotics.models.matrix import Matrix
from pybotics.models.optimizable import Optimizable
from pybotics.models.vector import Vector
from pybotics.utilities.constant import Constant
from pybotics.utilities.validation import is_4x4_ndarray, is_1d_ndarray


class Tool(Frame):
    """Tool class."""

    def __init__(self, matrix: np.ndarray = None, mass: float = 0, cg: np.ndarray = np.zeros(3)) -> None:
        super().__init__(matrix)
        self.mass = mass

        self._cg = None
        self.cg = cg

    @property
    def cg(self):
        return self._cg

    @cg.setter
    def cg(self, value):
        if is_1d_ndarray(value, Constant.POSITION_VECTOR_LENGTH.value):
            self._cg = value
        else:
            raise ValueError('cg must be a vector with len()==3')

    @property
    def tcp_position(self):
        return self.matrix[:-1, -1]

    @tcp_position.setter
    def tcp_position(self, value):
        self.matrix[:-1, -1] = value
