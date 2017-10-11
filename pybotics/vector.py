from collections import Sized

import numpy as np  # type: ignore
from abc import ABC, abstractmethod


class Vector(Sized):
    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        pass

    @vector.setter  # type: ignore
    @abstractmethod
    def vector(self, value: np.ndarray) -> None:
        pass

    def __len__(self):
        return len(self.vector)
