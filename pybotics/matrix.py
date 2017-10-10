from abc import ABC, abstractmethod
import numpy as np  # type: ignore


class Matrix(ABC):
    @property
    @abstractmethod
    def matrix(self) -> np.ndarray:
        pass

    @matrix.setter  # type: ignore
    @abstractmethod
    def matrix(self, value: np.ndarray) -> None:
        pass
