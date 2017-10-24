from abc import abstractmethod
from collections import Sized

import numpy as np  # type: ignore


class Vector(Sized):
    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        pass

    def __len__(self) -> int:
        # TODO: remove int() when mypy supports numpy
        return int(self.vector.size)
