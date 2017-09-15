from abc import ABC, abstractmethod

from pybotics.kinematics.convention import Convention
from pybotics.kinematics.kinematic_pair import KinematicPair
from pybotics.models.vector import Vector


class Link(Vector):
    @property
    @abstractmethod
    def vector(self):
        pass

    @abstractmethod
    def displace(self, position):
        pass

    def __init__(self) -> None:
        super().__init__()
        self.convention = Convention.UNDEFINED
        self.kinematic_pair = KinematicPair.UNDEFINED
