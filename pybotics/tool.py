"""Tool module."""

import numpy as np  # type: ignore

from pybotics.constant import Constant
from pybotics.frame import Frame
from pybotics.validation import is_1d_ndarray


class Tool(Frame):
    """Tool class."""

    def __init__(self, matrix: np.ndarray = None, mass: float = 0,
                 cg: np.ndarray = np.zeros(3)) -> None:
        super().__init__(matrix)
        self._cg = None

        self.mass = mass
        self.cg = cg

    @property
    def cg(self) -> np.ndarray:
        return self._cg

    @cg.setter
    def cg(self, value: np.ndarray) -> None:
        if is_1d_ndarray(value, Constant.POSITION_VECTOR_LENGTH.value):
            self._cg = value
        else:
            raise ValueError('cg must be a ndarray with len==3')

    @property
    def tcp_position(self) -> np.ndarray:
        return self.matrix[:-1, -1]

    @tcp_position.setter
    def tcp_position(self, value: np.ndarray) -> None:
        self.matrix[:-1, -1] = value
