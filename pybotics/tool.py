"""Tool module.

isort:skip_file
"""
from typing import Sequence, Union

import attr
import numpy as np  # type: ignore

from pybotics.geometry import matrix_2_vector, position_from_matrix, vector_2_matrix


@attr.s
class Tool:
    """Tool class."""

    matrix = attr.ib(factory=lambda: np.eye(4), type=np.ndarray)  # type: ignore
    mass = attr.ib(0, type=float)
    cg = attr.ib(factory=lambda: np.zeros(3), type=np.ndarray)  # type: ignore

    @property
    def position(self) -> Union[Sequence[float], np.ndarray]:
        """
        Get the position XYZ of the frame.

        :return:
        """
        return position_from_matrix(self.matrix)

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
