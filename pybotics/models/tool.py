"""Tool module."""
from typing import Union

import numpy as np

from pybotics.models.frame import Frame
from pybotics.utilities.validation import is_4x4_ndarray, is_1d_ndarray


class Tool:
    """Tool class."""

    def __init__(self,
                 tcp: Union[Frame, np.ndarray] = None,
                 mass: float = 0,
                 cg: np.ndarray = np.zeros(3)) -> None:
        """
        Construct Tool object.

        :param tcp: tool center point 4x4 transform
        :param mass: mass of tool [kg]
        :param cg: center of gravity xyz position [mm]
        """

        if is_4x4_ndarray(tcp):
            tcp = Frame(tcp)

        if not is_1d_ndarray(cg, 3):
            raise ValueError('cg must be 1D vector of length 3')

        self.tcp = Frame() if tcp is None else tcp
        self.mass = mass
        self.cg = cg

    @property
    def tcp_position(self):
        return self.tcp.matrix[:-1, -1]

    @tcp_position.setter
    def tcp_position(self, value):
        self.tcp.matrix[:-1, -1] = value
