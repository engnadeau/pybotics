from abc import ABC, abstractmethod


class Matrix(ABC):
    @property
    @abstractmethod
    def matrix(self):
        pass

    @matrix.setter
    @abstractmethod
    def matrix(self, value):
        pass
