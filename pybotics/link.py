"""Link module."""
from abc import abstractmethod

import numpy as np  # type: ignore

from pybotics.kinematic_pair import KinematicPair
from pybotics.link_convention import LinkConvention


class Link:
    """Links: connected joints allowing relative motion of neighboring link."""

    @abstractmethod
    def displace(self, position: float) -> np.ndarray:
        """
        Generate a vector of the new link state given a displacement.

        :param position: given displacement
        :return vector of new displacement state
        """
        pass

    @property
    @abstractmethod
    def convention(self) -> LinkConvention:
        """
        Get the LinkConvention.

        :return: link convention
        """
        pass

    @property
    @abstractmethod
    def kinematic_pair(self) -> KinematicPair:
        """
        Get the KinematicPair.

        :return: kinematic pair
        """
        pass

    @abstractmethod
    def transform(self, position: float = 0) -> np.ndarray:
        """
        Generate a 4x4 transform matrix given a displacement.

        :param position: given displacement
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
