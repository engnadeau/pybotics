import numpy as np

from pybotics.exceptions import PybotException
from pybotics.types import Vector


class Tool:
    def __init__(self,
                 tcp: np.ndarray = np.eye(4),
                 mass: float = 0,
                 cg: np.ndarray = np.zeros(3)):
        self.tcp = tcp
        self.mass = mass
        self.cg = cg

    def tcp_xyz(self, xyz: Vector):
        """Set the tool center point (TCP) xyz position.

        :param xyz: position [mm]
        :return:
        """
        if len(xyz) != 3:
            raise PybotException
        for i, parameter in enumerate(xyz):
            self.tcp[i, -1] = parameter
