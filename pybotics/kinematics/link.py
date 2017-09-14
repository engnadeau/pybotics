from abc import ABC, abstractmethod

from pybotics.kinematics.convention import Convention
from pybotics.kinematics.kinematic_pair import KinematicPair


class Link(ABC):
    def __init__(self) -> None:
        super().__init__()
        self.convention = Convention.UNDEFINED
        self.kinematic_pair = KinematicPair.UNDEFINED

    @abstractmethod
    def vector(self, position=0):
        pass

    @abstractmethod
    def transform(self, position=0):
        pass
