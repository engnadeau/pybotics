"""Tool module.

isort:skip_file
"""

import attr
import numpy as np
import numpy.typing as npt


from pybotics.geometry import matrix_2_vector, position_from_matrix, vector_2_matrix


@attr.s
class Tool:
    """Tool class."""

    matrix = attr.ib(factory=lambda: np.eye(4), type=npt.NDArray[np.float64])
    mass = attr.ib(0, type=float)
    cg = attr.ib(factory=lambda: np.zeros(3), type=npt.NDArray[np.float64])

    @property
    def position(self) -> npt.NDArray[np.float64]:
        """
        Get the position XYZ of the frame.

        :return:
        """
        return position_from_matrix(self.matrix)

    @position.setter
    def position(self, value: npt.NDArray[np.float64]) -> None:
        self.matrix[:-1, -1] = value

    @property
    def vector(self) -> npt.NDArray[np.float64]:
        """
        Return the vector representation of the frame as EULER ZYX.

        :return: vectorized frame
        """
        return matrix_2_vector(self.matrix)

    @vector.setter
    def vector(self, value: npt.NDArray[np.float64]) -> None:
        self.matrix = vector_2_matrix(value)
