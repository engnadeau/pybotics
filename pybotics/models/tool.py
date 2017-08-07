"""Tool module."""
import numpy as np

from pybotics.data_validation import validate_1d_vector, validate_4x4_matrix


class Tool:
    """Tool class."""

    def __init__(self,
                 tcp: np.ndarray = np.eye(4),
                 mass: float = 0,
                 cg: np.ndarray = np.zeros(3)) -> None:
        """
        Construct Tool object.

        :param tcp: tool center point 4x4 transform
        :param mass: mass of tool [kg]
        :param cg: center of gravity xyz position [mm]
        """
        # validate input
        validate_4x4_matrix(tcp)
        validate_1d_vector(cg, 3)

        self.tcp = tcp
        self.mass = mass
        self.cg = cg

    @property
    def tcp_xyz(self) -> np.ndarray:
        return self.tcp[:-1, -1]

    @tcp_xyz.setter
    def tcp_xyz(self, value: np.ndarray) -> None:
        """
        Set the tool center point (TCP) xyz position.

        :param xyz: position [mm]
        :return:
        """
        validate_1d_vector(value, 3)
        self.tcp[:-1, -1] = value
