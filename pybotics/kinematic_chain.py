"""Kinematic chain module."""
import logging
from abc import abstractmethod
from typing import Optional, Sequence, Sized, Union

import numpy as np  # type: ignore

from pybotics.errors import PyboticsError
from pybotics.link import Link, MDHLink, RevoluteMDHLink


class KinematicChain(Sized):
    """
    An assembly of rigid bodies connected by joints to provide constrained
    (or desired) motion that is the mathematical model for a mechanical system.
    """

    @property
    @abstractmethod
    def links(self) -> Sequence[Link]:
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


class MDHKinematicChain(KinematicChain):
    @property
    def links(self) -> Sequence[Link]:
        return self._links

    def __init__(self,
                 links: Union[Sequence[MDHLink], np.ndarray]
                 ) -> None:
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
                logging.getLogger(__name__).error(e)
                raise PyboticsError(
                    'MDH links have {} parameters per link.'.format(
                        MDHLink.size))

            # build links
            # assume revolute joints
            self._links = [RevoluteMDHLink(*x) for x in links]
        else:
            self._links = links

    def __len__(self) -> int:
        return len(self._links)

    @property
    def num_parameters(self) -> int:
        return len(self) * MDHLink.size

    def transforms(self, q: Optional[Sequence[float]] = None) -> \
            Sequence[np.ndarray]:
        q = np.zeros(len(self)) if q is None else q
        transforms = [link.transform(p) for link, p in
                      zip(self._links, q)]  # type: ignore
        return transforms

    @property
    def vector(self) -> np.ndarray:
        return np.array([l.vector for l in self._links]).ravel()
