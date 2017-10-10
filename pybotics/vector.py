import numpy as np  # type: ignore
from abc import ABC, abstractmethod


class Vector(ABC):
    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        pass

    @vector.setter  # type: ignore
    @abstractmethod
    def vector(self, value: np.ndarray) -> None:
        pass
