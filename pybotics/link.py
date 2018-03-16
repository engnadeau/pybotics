"""Link module."""
from abc import abstractmethod

import numpy as np  # type: ignore

import pybotics.conventions as conv


class Link:
    """Links: connected joints allowing relative motion of neighboring link."""

    @abstractmethod
    def displace(self, q: float) -> np.ndarray:
        """
        Generate a vector of the new link state given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        pass

    @property
    @abstractmethod
    def convention(self) -> conv.Link:
        """
        Get the LinkConvention.

        :return: link convention
        """
        pass

    @property
    @abstractmethod
    def kinematic_pair(self) -> conv.KinematicPair:
        """
        Get the KinematicPair.

        :return: kinematic pair
        """
        pass

    @abstractmethod
    def transform(self, q: float = 0) -> np.ndarray:
        """
        Generate a 4x4 transform matrix given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        pass

    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        """
        Return the vector representation of the link.

        :return: vectorized kinematic chain
        """
        pass


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
    def convention(self) -> conv.Link:
        """
        Get the LinkConvention.

        :return: link convention used
        """
        return conv.Link.MDH

    def transform(self, q: float = 0) -> np.ndarray:
        """
        Generate a 4x4 transform matrix with a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        vector = self.displace(q)

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
        ], dtype=float)


class RevoluteMDHLink(MDHLink):
    """
    Link class that uses Modified DH parameters for a revolute joint.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    def __init__(self, alpha: float, a: float, theta: float, d: float) -> None:
        """
        Construct a revolute MDH link instance.

        :param alpha:
        :param a:
        :param theta:
        :param d:
        """
        super().__init__(alpha, a, theta, d)

    def displace(self, q: float = 0) -> np.ndarray:
        """
        Generate a vector of the new link state given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        vector = self.vector
        vector[2] += q
        return vector

    @property
    def kinematic_pair(self) -> conv.KinematicPair:
        """
        Kinematic pair used.

        :return: kinematic pair
        """
        return conv.KinematicPair.REVOLUTE
