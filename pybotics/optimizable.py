from abc import ABC, abstractmethod
from typing import List

import numpy as np  # type:ignore


class Optimizable(ABC):
    def __init__(self) -> None:
        self._optimization_mask = [False]

    @property
    @abstractmethod
    def optimization_vector(self) -> np.ndarray:
        pass

    @property
    @abstractmethod
    def optimization_mask(self) -> List[bool]:
        pass
