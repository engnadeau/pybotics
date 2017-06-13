"""Tool module."""
import numpy as np  # type: ignore
from pybotics.exceptions import PybotException


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
        self.tcp = tcp
        self.mass = mass
        self.cg = cg

    def tcp_xyz(self, xyz: np.ndarray) -> None:
        """
        Set the tool center point (TCP) xyz position.

        :param xyz: position [mm]
        :return:
        """
        if len(xyz) != 3:
            raise PybotException
        for i, parameter in enumerate(xyz):
            self.tcp[i, -1] = parameter
