"""MDH link module."""

import numpy as np  # type: ignore

from pybotics.link import Link
from pybotics.link_convention import LinkConvention


# noinspection PyAbstractClass
class MDHLink(Link):
    """
    Link class that uses Modified DH parameters.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    def __init__(self, alpha: float, a: float, theta: float, d: float) -> None:
        """
        Construct a MDH link.

        :param alpha:
        :param a:
        :param theta:
        :param d:
        """
        self.alpha = alpha
        self.a = a
        self.theta = theta
        self.d = d

    @property
    def convention(self) -> LinkConvention:
        """
        Get the LinkConvention.

        :return: link convention used
        """
        return LinkConvention.MDH

    def transform(self, position: float = 0) -> np.ndarray:
        """
        Generate a 4x4 transform matrix with a displacement.

        :param position: given displacement
        :return vector of new displacement state
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

        transform = np.array([
            [crz, -srz, 0, a],
            [crx * srz, crx * crz, -srx, -d * srx],
            [srx * srz, crz * srx, crx, d * crx],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        return transform

    @property
    def vector(self) -> np.ndarray:
        """
        Return the vector representation of the link.

        :return: vectorized kinematic chain
        """
        return np.array([
            self.alpha,
            self.a,
            self.theta,
            self.d
        ])
