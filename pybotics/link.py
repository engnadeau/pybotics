from abc import abstractmethod

import numpy as np  # type: ignore

from pybotics.convention import Convention
from pybotics.kinematic_pair import KinematicPair
from pybotics.vector import Vector


class Link(Vector):
    def __init__(self) -> None:
        self._convention = Convention.UNDEFINED

    @abstractmethod
    def transform(self, position: float) -> np.ndarray:
        pass

    @abstractmethod
    def displace(self, position: float) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def convention(self) -> Convention:
        pass

    @property
    @abstractmethod
    def kinematic_pair(self) -> KinematicPair:
        pass
