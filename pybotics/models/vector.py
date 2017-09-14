from abc import ABC, abstractmethod


class Vector(ABC):
    @property
    @abstractmethod
    def vector(self):
        pass

    @vector.setter
    @abstractmethod
    def vector(self, value):
        pass
