"""Link module."""
from abc import abstractmethod
from collections.abc import Sized
from typing import Sequence, Union

import attr
import numpy as np  # type: ignore

from pybotics.json_encoder import JSONEncoder


@attr.s
class Link(Sized):
    """Links: connected joints allowing relative motion of neighboring link."""

    def to_json(self) -> str:
        """Encode model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    def __len__(self) -> int:
        """Get number of parameters."""
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
        """Get number of parameters."""
        raise NotImplementedError


@attr.s
class MDHLink(Link):
    """
    Link class that uses Modified DH parameters.

    https://en.wikipedia.org/wiki/Denavit%E2%80%93Hartenberg_parameters
    """

    _size = 4
    alpha = attr.ib(0, type=float)
    a = attr.ib(0, type=float)
    theta = attr.ib(0, type=float)
    d = attr.ib(0, type=float)

    @property
    def size(self) -> int:
        """Get number of parameters."""
        return self._size

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

        transform = np.array(
            [
                [crz, -srz, 0, a],
                [crx * srz, crx * crz, -srx, -d * srx],
                [srx * srz, crz * srx, crx, d * crx],
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )

        return transform

    @property
    def vector(self) -> np.ndarray:
        """
        Return the vector representation of the link.

        :return: vectorized kinematic chain
        """
        return np.array([self.alpha, self.a, self.theta, self.d], dtype=float)

    # noinspection PyMethodOverriding
    @vector.setter
    def vector(self, value: Sequence[float]) -> None:
        """Set parameters."""
        self.alpha = value[0]
        self.a = value[1]
        self.theta = value[2]
        self.d = value[3]


@attr.s
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


@attr.s
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
