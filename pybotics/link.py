"""Link module."""
from typing import Sequence, Union

import numpy as np  # type: ignore
from abc import abstractmethod
from collections import Sized


class Link(Sized):
    """Links: connected joints allowing relative motion of neighboring link."""

    def __len__(self) -> int:
        return self.size

    @abstractmethod
    def displace(self, q: float) -> Union[Sequence[float], np.ndarray]:
        """
        Generate a vector of the new link state given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        raise NotImplementedError

    @abstractmethod
    def transform(self, q: float = 0) -> np.ndarray:
        """
        Generate a 4x4 transform matrix given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        """
        Return the vector representation of the link.

        :return: vectorized kinematic chain
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def size(self) -> int:
        raise NotImplementedError


# noinspection PyAbstractClass
class MDHLink(Link):
    """
    Link class that uses Modified DH parameters.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    _size = 4

    @property
    def size(self) -> int:
        return self._size

    def __init__(self,
                 alpha: float = 0,
                 a: float = 0,
                 theta: float = 0,
                 d: float = 0
                 ) -> None:
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

    # noinspection PyMethodOverriding
    @vector.setter
    def vector(self, value: Sequence[float]) -> None:
        self.alpha = value[0]
        self.a = value[1]
        self.theta = value[2]
        self.d = value[3]


class RevoluteMDHLink(MDHLink):
    """
    Link class that uses Modified DH parameters for a revolute joint.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    def displace(self, q: float = 0) -> Union[Sequence[float], np.ndarray]:
        """
        Generate a vector of the new link state given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        v = np.copy(self.vector)
        v[2] += q
        return v


class PrismaticMDHLink(MDHLink):
    """
    Link class that uses Modified DH parameters for a revolute joint.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    def displace(self, q: float = 0) -> Union[Sequence[float], np.ndarray]:
        """
        Generate a vector of the new link state given a displacement.

        :param q: given displacement
        :return vector of new displacement state
        """
        v = np.copy(self.vector)
        v[3] += q
        return v
