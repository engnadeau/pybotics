from copy import deepcopy

import numpy as np  # type: ignore

from pybotics.kinematic_pair import KinematicPair
from pybotics.mdh_link import MDHLink


class RevoluteMDHLink(MDHLink):
    @property
    def kinematic_pair(self) -> KinematicPair:
        return KinematicPair.REVOLUTE

    def __init__(self, alpha: float, a: float, theta: float, d: float) -> None:
        super().__init__(alpha, a, theta, d)
        self._kinematic_pair = KinematicPair.REVOLUTE

    def displace(self, position: float = 0) -> np.ndarray:
        vector = deepcopy(self.vector)
        vector[2] += position
        return vector
