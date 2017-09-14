import numpy as np

from pybotics.kinematics.kinematic_pair import KinematicPair
from pybotics.kinematics.mdh_link import MDHLink


class RevoluteMDHLink(MDHLink):
    def __init__(self, alpha, a, theta, d) -> None:
        super().__init__(alpha, a, theta, d)
        self._kinematic_pair = KinematicPair.REVOLUTE

    def vector(self, position=0):
        return np.array([
            self.alpha,
            self.a,
            self.theta + position,
            self.d
        ])
