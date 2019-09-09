"""Kinematic chain module."""
import logging
from abc import abstractmethod
from typing import Any, Optional, Sequence, Sized, Union

import attr
import numpy as np  # type: ignore

from pybotics.errors import PyboticsError
from pybotics.json_encoder import JSONEncoder
from pybotics.link import Link, MDHLink, RevoluteMDHLink

# set logging
logger = logging.getLogger(__name__)


@attr.s
class KinematicChain(Sized):
    """
    An assembly of rigid bodies connected by joints.

    Provides constrained (or desired) motion that is the
    mathematical model for a mechanical system.
    """

    def to_json(self) -> str:
        """Encode model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    @property  # type: ignore
    @abstractmethod
    def matrix(self) -> np.ndarray:
        """
        Convert chain to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

    @matrix.setter  # type: ignore
    @abstractmethod
    def matrix(self, value: np.ndarray) -> None:
        """
        Set to matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        raise NotImplementedError

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
    def num_parameters(self) -> int:
        """Get number of parameters of all links."""
        # noinspection PyProtectedMember
        raise NotImplementedError

    @abstractmethod
    def transforms(self, q: Optional[Sequence[float]] = None) -> Sequence[np.ndarray]:
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


def _validate_links(value: Union[Sequence[MDHLink], np.ndarray]) -> Sequence[MDHLink]:
    if isinstance(value, np.ndarray):
        try:
            value = value.reshape((-1, MDHLink._size))
        except ValueError as e:
            logger.error(str(e))
            raise PyboticsError(f"MDH links have {MDHLink.size} parameters per link.")

        # FIXME: only assumes revolute joints
        value = [RevoluteMDHLink(*x) for x in value]
    return value


@attr.s
class MDHKinematicChain(KinematicChain):
    """Kinematic Chain of MDH links."""

    _links = attr.ib(type=Union[Sequence[MDHLink], np.ndarray])

    def __attrs_post_init__(self) -> None:
        """Post-attrs initialization."""
        self._links = _validate_links(self._links)

    @classmethod
    def from_parameters(cls: Any, parameters: Sequence[float]) -> Any:
        """Construct Kinematic Chain from parameters."""
        kc = cls(parameters)
        return kc

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
        Set matrix of link parameters.

        Rows = links
        Columns = parameters
        """
        for i, v in enumerate(value):
            self.links[i].vector = v

    @property
    def links(self) -> Sequence[MDHLink]:
        """Get links."""
        x = self._links  # type: Sequence[MDHLink]
        return x

    @links.setter
    def links(self, value: Union[Sequence[MDHLink], np.ndarray]) -> None:
        """Set links."""
        self._links = _validate_links(value)

    def __len__(self) -> int:
        """Get ndof."""
        return len(self._links)

    @property
    def num_parameters(self) -> int:
        """Get number of parameters of all links."""
        # noinspection PyProtectedMember
        return len(self) * MDHLink._size

    def transforms(self, q: Optional[Sequence[float]] = None) -> Sequence[np.ndarray]:
        """Get sequence of 4x4 transforms."""
        q = np.zeros(len(self)) if q is None else q
        transforms = [link.transform(p) for link, p in zip(self._links, q)]
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
