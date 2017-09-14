from abc import ABC, abstractmethod


class Optimizable(ABC):
    def __init__(self):
        self._optimization_mask = False

    @property
    @abstractmethod
    def optimization_vector(self):
        pass

    @optimization_vector.setter
    @abstractmethod
    def optimization_vector(self, value):
        pass

    @property
    @abstractmethod
    def optimization_mask(self):
        pass

    @optimization_mask.setter
    @abstractmethod
    def optimization_mask(self, value):
        pass
