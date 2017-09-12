from abc import ABC, abstractmethod


class Link(ABC):
    @abstractmethod
    def vector(self, position=0):
        pass

    @abstractmethod
    def transform(self, position=0):
        pass
