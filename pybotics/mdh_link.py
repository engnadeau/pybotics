from abc import abstractmethod
from typing import Sequence

import numpy as np  # type: ignore

from pybotics.convention import Convention
from pybotics.link import Link


class MDHLink(Link):
    @property
    def convention(self) -> Convention:
        return Convention.MDH

    @abstractmethod
    def displace(self, position: float) -> np.ndarray:
        pass

    @property
    def vector(self) -> np.ndarray:
        return np.array([
            self.alpha,
            self.a,
            self.theta,
            self.d
        ])

    @vector.setter
    def vector(self, value: Sequence) -> None:
        self.alpha = value[0]
        self.a = value[1]
        self.theta = value[2]
        self.d = value[3]

    def __init__(self, alpha: float, a: float, theta: float, d: float) -> None:
        super().__init__()
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d

    def transform(self, position: float = 0) -> np.ndarray:
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
