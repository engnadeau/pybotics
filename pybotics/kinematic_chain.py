"""Kinematic chain module."""
import logging
from abc import abstractmethod
from typing import Dict, Optional, Sequence, Sized, Union

import numpy as np  # type: ignore

from pybotics.errors import PyboticsError
from pybotics.json_encoder import JSONEncoder
from pybotics.link import Link, MDHLink, RevoluteMDHLink


class KinematicChain(Sized):
    """
    An assembly of rigid bodies connected by joints.

    Provides constrained (or desired) motion that is the
    mathematical model for a mechanical system.
    """

    def __repr__(self) -> str:
        """Encode model as JSON."""
        return self.to_json()

    def to_json(self) -> str:
        """Encode model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    @property
    def matrix(self) -> np.ndarray:
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

    @matrix.setter
    def matrix(self, value: np.ndarray) -> None:
        """
        Set to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

    def to_dict(self) -> Dict[str, Dict[str, float]]:
        """Convert chain to dict."""
        return {
            'link_{}'.format(i): e.to_dict() for i, e in enumerate(self.links)
        }

    @property
    @abstractmethod
    def links(self) -> Sequence[Link]:
        """Get links."""
        raise NotImplementedError

    @property
    def ndof(self) -> int:
        """
        Get number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self)

    @property
    @abstractmethod
    def num_parameters(self) -> int:
        """
        Get the number of kinematic parameters.

        :return: number of degrees of freedom
        """
        raise NotImplementedError

    @abstractmethod
    def transforms(self, q: Optional[Sequence[float]] = None) -> \
            Sequence[np.ndarray]:
        """
        Generate a sequence of spatial transforms.

        The sequence represents the given position of the kinematic chain.
        :param q: given position of kinematic chain
        :return: sequence of transforms
        """
        raise NotImplementedError

    @property
    @abstractmethod
    def vector(self) -> np.ndarray:
        """
        Get the vector representation of the kinematic chain.

        :return: vectorized kinematic chain
        """
        raise NotImplementedError

    @vector.setter
    def vector(self, value: Sequence[float]) -> None:
        """Set parameters of all links."""
        raise NotImplementedError


class MDHKinematicChain(KinematicChain):
    """Kinematic Chain of MDH links."""

    @property
    def matrix(self) -> np.ndarray:
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        return np.array([l.vector for l in self._links])

    @matrix.setter
    def matrix(self, value: np.ndarray) -> None:
        """
        Set to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        for i, v in enumerate(value):
            self.links[i].vector = v

    @property
    def links(self) -> Sequence[MDHLink]:
        """Get links."""
        return self._links

    def __init__(self,
                 links: Union[Sequence[MDHLink], np.ndarray]
                 ) -> None:
        """Init chain."""
        super().__init__()
        # set links
        if isinstance(links, np.ndarray):
            # we have an array of parameters
            # validate input
            # should be 1D or 2D
            try:
                # noinspection PyProtectedMember
                links = links.reshape((-1, MDHLink._size))
            except ValueError as e:
                logging.getLogger(__name__).error(str(e))
                raise PyboticsError(
                    'MDH links have {} parameters per link.'.format(
                        MDHLink.size))

            # build links
            # assume revolute joints
            self._links = [
                RevoluteMDHLink(*x) for x in links
            ]  # type: Sequence[MDHLink]
        else:
            self._links = links  # type: Sequence[MDHLink]

    def __len__(self) -> int:
        """Get ndof."""
        return len(self._links)

    @property
    def num_parameters(self) -> int:
        """Get number of parameters of all links."""
        # noinspection PyProtectedMember
        return len(self) * MDHLink._size

    def transforms(self,
                   q: Optional[Sequence[float]] = None
                   ) -> Sequence[np.ndarray]:
        """Get sequency of 4x4 transforms."""
        q = np.zeros(len(self)) if q is None else q
        transforms = [link.transform(p) for link, p in
                      zip(self._links, q)]  # type: ignore
        return transforms

    @property
    def vector(self) -> np.ndarray:
        """Get parameters of all links."""
        return self.matrix.ravel()

    # noinspection PyMethodOverriding
    @vector.setter
    def vector(self, value: Sequence[float]) -> None:
        """Set parameters of all links."""
        # noinspection PyProtectedMember
        value = np.array(value).reshape((-1, MDHLink._size))
        self.matrix = value
