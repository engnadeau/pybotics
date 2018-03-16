"""Kinematic chain module."""
from itertools import compress
from typing import Any, Sequence, Union, Sized, Optional

import numpy as np  # type: ignore

import pybotics.conventions as conv
import pybotics.link as lk


class KinematicChain(Sized):
    """
    Assembly of rigid bodies.

    Connected by joints to provide constrained motion.
    """

    def __init__(self, links: Sequence[lk.Link]) -> None:
        """
        Construct a kinematic chain.

        :param links: sequence of links
        """
        self._links = []  # type: Sequence[Link]
        self._optimization_mask = [False] * len(links)

        self.links = links

    def __len__(self) -> int:
        """
        Get the length of the chain.

        :return:
        """
        return len(self.links)

    @staticmethod
    def array_2_links(
            array: np.ndarray,
            link_convention: conv.Link = conv.Link.MDH,
            kinematic_pairs:
            Union[conv.KinematicPair,
                  Sequence[
                      conv.KinematicPair]] = conv.KinematicPair.REVOLUTE
    ) -> Sequence[lk.Link]:
        """
        Generate a sequence of links from a given array of link parameters.

        :param kinematic_pairs:
        :param array: link parameters
        :param link_convention: link convention
        :return: sequence of links
        """
        # validate
        if link_convention in conv.Link:
            # vectors are reshaped to a 2D array based
            # on number of parameters per link
            array = array.reshape((-1, link_convention.value))

            # turn single KinematicPair into sequence
            if isinstance(kinematic_pairs, conv.KinematicPair):
                kinematic_pairs = [kinematic_pairs] * len(array)

            # create link sequences based on convention;
            links = []
            for row, _ in zip(array, kinematic_pairs):
                links.append(lk.RevoluteMDHLink(*row))
        else:
            raise NotImplementedError(link_convention)

        return links

    @property
    def convention(self) -> conv.Link:
        """
        Get LinkConvention.

        :return: link convention
        """
        return self.links[0].convention

    @classmethod
    def from_array(
            cls, array: np.ndarray,
            link_convention: conv.Link = conv.Link.MDH,
            kinematic_pairs: Union[
                conv.KinematicPair,
                Sequence[
                    conv.KinematicPair]] = conv.KinematicPair.REVOLUTE) -> Any:
        """
        Generate a kinematic chain from a given array of link parameters.

        :param array: link parameters
        :param link_convention:
        :param kinematic_pairs:
        :return: kinematic chain instance
        """
        return cls(cls.array_2_links(array, link_convention, kinematic_pairs))

    @property
    def links(self) -> Sequence[lk.Link]:
        """
        Get links of the kinematic chain.

        :return: sequence of links
        """
        return self._links

    @links.setter
    def links(self, value: Sequence[lk.Link]) -> None:
        self._links = value

    @property
    def num_dof(self) -> int:
        """
        Get number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self)

    @property
    def num_parameters(self) -> int:
        """
        Get the number of kinematic parameters.

        :return: number of degrees of freedom
        """
        return len(self) * self.convention.value

    @property
    def optimization_mask(self) -> Sequence[bool]:
        """
        Get the mask used to select the optimization parameters.

        :return: mask
        """
        return self._optimization_mask

    @optimization_mask.setter
    def optimization_mask(self,
                          mask: Union[bool, Sequence[bool]]) -> None:
        if isinstance(mask, bool):
            self._optimization_mask = [mask] * len(self.vector)
        else:
            self._optimization_mask = list(mask)

    @property
    def optimization_vector(self) -> np.ndarray:
        """
        Get the values of parameters being optimized.

        :return: optimization parameter values
        """
        filtered_iterator = compress(self.vector, self.optimization_mask)
        optimization_vector = np.array(list(filtered_iterator))
        return optimization_vector

    def transforms(self, q: Optional[Sequence[float]] = None) -> \
            Sequence[np.ndarray]:
        """
        Generate a sequence of spatial transforms.

        The sequence represents the given position of the kinematic chain.
        :param q: given position of kinematic chain
        :return: sequence of transforms
        """
        # validate
        if q is None:
            q = np.zeros(len(self))

        # FIXME: remove type ignore for mypy bugs:
        # error: Argument 2 to "zip" has incompatible type
        # "Optional[Iterable[float]]"; expected "Iterable[float]";
        # error: Call to untyped function "transform"
        # of "Link" in typed context
        transforms = [link.transform(p) for link, p in
                      zip(self.links, q)]  # type: ignore
        return transforms

    @property
    def vector(self) -> np.ndarray:
        """
        Get the vector representation of the kinematic chain.

        :return: vectorized kinematic chain
        """
        link_vectors = [link.vector for link in self.links]
        v = np.array(link_vectors).ravel()
        return v
