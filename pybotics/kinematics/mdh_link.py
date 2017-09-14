from abc import abstractmethod

import numpy as np

from pybotics.kinematics.convention import Convention
from pybotics.kinematics.link import Link


class MDHLink(Link):
    @property
    def vector(self):
        return np.array([
            self.alpha,
            self.a,
            self.theta,
            self.d
        ])

    @abstractmethod
    def displace(self, position):
        pass

    def __init__(self, alpha, a, theta, d) -> None:
        super().__init__()
        self.convention = Convention.MDH
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d

    def transform(self, position=0) -> np.ndarray:
        """Return the Modified Denavit-Hartenberg (MDH) 4x4 matrix for a robot link (Craig 1986).

        Angular arguments are in radians.
        Calling forward_transform(rx,tx,tz,rz) is the same as using rotx(rx)*transl(tx,0,tx)*rotz(rz)

        :param position:
        :param mdh_parameters: list of MDH paramters: alpha [rad], a [mm], theta [rad], d [mm]
        :return: 4x4 transform
        """

        vector = self.displace(position)

        alpha = vector[0]
        a = vector[1]
        theta = vector[2]
        d = vector[3]

        crx = np.cos(alpha)
        srx = np.sin(alpha)
        crz = np.cos(theta)
        srz = np.sin(theta)

        return np.array([
            [crz, -srz, 0, a],
            [crx * srz, crx * crz, -srx, -d * srx],
            [srx * srz, crz * srx, crx, d * crx],
            [0, 0, 0, 1]
        ], dtype=np.float64)
