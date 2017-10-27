"""Tool module."""

import numpy as np  # type: ignore

from pybotics.constants import POSITION_VECTOR_LENGTH
from pybotics.errors import SequenceLengthError
from pybotics.frame import Frame
from pybotics.validation import is_1d_ndarray


class Tool(Frame):
    """Tool class."""

    def __init__(self, matrix: np.ndarray = None, mass: float = 0,
                 cg: np.ndarray = np.zeros(3)) -> None:
        """
        Construct tool instance.

        :param matrix: 4x4 transform matrix of tool frame
        :param mass: mass of tool located at CG
        :param cg: centre of gravity
        """
        super().__init__(matrix)
        self._cg = None

        self.cg = cg
        self.mass = mass

    @property
    def cg(self) -> np.ndarray:
        """
        Centre of gravity.

        :return: centre of gravity [x,y,z]
        """
        return self._cg

    @cg.setter
    def cg(self, value: np.ndarray) -> None:
        if is_1d_ndarray(value, POSITION_VECTOR_LENGTH):
            self._cg = value
        else:
            raise SequenceLengthError('value', POSITION_VECTOR_LENGTH)

    @property
    def tcp_position(self) -> np.ndarray:
        """
        Tool centre point (TCP) position.

        with respect to the root of the frame (i.e., robot flange)
        :return: tcp [x,y,z]
        """
        return self.matrix[:-1, -1]

    @tcp_position.setter
    def tcp_position(self, value: np.ndarray) -> None:
        if is_1d_ndarray(value, POSITION_VECTOR_LENGTH):
            self.matrix[:-1, -1] = value
        else:
            raise SequenceLengthError('value', POSITION_VECTOR_LENGTH)
