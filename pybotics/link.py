from abc import abstractmethod

from pybotics.convention import Convention
from pybotics.kinematic_pair import KinematicPair
from pybotics.vector import Vector
import numpy as np  # type: ignore


class Link(Vector):
    def __init__(self):
        self._convention = Convention.UNDEFINED

    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        pass

    @abstractmethod
    def displace(self, position: float) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def convention(self) -> Convention:
        pass

    @convention.setter
    def convention(self, value: Convention):
        pass

    @property
    @abstractmethod
    def kinematic_pair(self) -> KinematicPair:
        pass
