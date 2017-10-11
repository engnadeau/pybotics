from typing import Union

import numpy as np  # type:ignore
from abc import ABC, abstractmethod


class Optimizable(ABC):
    def __init__(self) -> None:
        self._optimization_mask = [False]

    @property
    @abstractmethod
    def optimization_vector(self) -> np.ndarray:
        pass

    @optimization_vector.setter  # type: ignore
    @abstractmethod
    def optimization_vector(self, value: np.ndarray) -> None:
        pass

    @property
    @abstractmethod
    def optimization_mask(self) -> List[bool]:
        pass

    @optimization_mask.setter  # type: ignore
    @abstractmethod
    def optimization_mask(self, value: Union[bool, List[bool]]) -> None:
        pass
