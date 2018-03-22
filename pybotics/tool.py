"""Tool module."""
from typing import Sequence, Optional, Union

import numpy as np  # type: ignore

from pybotics.constants import TRANSFORM_MATRIX_SHAPE, POSITION_VECTOR_LENGTH
from pybotics.errors import PyboticsError
from pybotics.geometry import matrix_2_vector, vector_2_matrix
from pybotics.validation import is_4x4_matrix, is_vector


class Tool:
    """Tool class."""

    def __init__(self,
                 matrix: Optional[np.ndarray] = None,
                 mass: float = 0,
                 cg: Optional[Sequence[float]] = None
                 ) -> None:
        """
        Construct tool instance.

        :param matrix: 4x4 transform matrix of tool frame
        :param mass: mass of tool located at CG
        :param cg: centre of gravity
        """
        self._matrix = None
        self.matrix = np.eye(TRANSFORM_MATRIX_SHAPE[0]) \
            if matrix is None else matrix
        self._cg = None
        self.cg = np.zeros(POSITION_VECTOR_LENGTH) \
            if cg is None else cg
        self.mass = mass

    @property
    def cg(self) -> Union[Sequence[float], np.ndarray]:
        """
        Centre of gravity.

        :return: centre of gravity [x,y,z]
        """
        return self._cg

    @cg.setter
    def cg(self, value: Sequence[float]) -> None:
        if not is_vector(value, POSITION_VECTOR_LENGTH):
            raise PyboticsError(
                'CG must be 1D vector if length {}.'.format(
                    POSITION_VECTOR_LENGTH)
            )
        self._cg = np.array(value)

    @property
    def matrix(self) -> np.ndarray:
        """
        Return the internal matrix representation of the frame.

        :return: 4x4 matrix
        """
        return self._matrix

    @matrix.setter
    def matrix(self, value: np.ndarray) -> None:
        if not is_4x4_matrix(value):
            raise PyboticsError('4x4 transform matrix is required.')
        self._matrix = value

    @property
    def position(self)-> Union[Sequence[float], np.ndarray]:
        """
        Get the position XYZ of the frame.

        :return:
        """
        return self.matrix[:-1, -1]

    @position.setter
    def position(self, value: Sequence[float]) -> None:
        self.matrix[:-1, -1] = value

    @property
    def vector(self) -> np.ndarray:
        """
        Return the vector representation of the frame as EULER ZYX.

        :return: vectorized frame
        """
        return matrix_2_vector(self.matrix)

    @vector.setter
    def vector(self, value: Sequence[float]) -> None:
        self.matrix = vector_2_matrix(value)
