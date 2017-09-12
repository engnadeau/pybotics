import numpy as np

from pybotics.kinematics.mdh_link import MDHLink


class RevoluteMDHLink(MDHLink):
    def vector(self, position=0):
        return np.array([
            self.alpha,
            self.a,
            self.theta + position,
            self.d
        ])
