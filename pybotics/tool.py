"""Tool module."""
from typing import Sequence

import numpy as np  # type: ignore

from pybotics.constants import POSITION_VECTOR_LENGTH
from pybotics.frame import Frame


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
    def cg(self, value: Sequence[float]) -> None:
        self._cg = np.array(value)
