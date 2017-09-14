import numpy as np

from pybotics.kinematics.convention import Convention
from pybotics.kinematics.kinematic_pair import KinematicPair
from pybotics.kinematics.link import Link


class DHLink(Link):
    def vector(self, position=0):
        return np.array([
            self.alpha,
            self.a,
            self.theta,
            self.d
        ])

    def __init__(self, alpha, a, theta, d) -> None:
        super().__init__()
        self.convention = Convention.DH
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d

    def transform(self, position=0) -> np.ndarray:
        raise NotImplementedError
