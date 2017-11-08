"""Revolute MDH link module."""
import numpy as np  # type: ignore

from pybotics.kinematic_pair import KinematicPair
from pybotics.mdh_link import MDHLink


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

    def displace(self, position: float = 0) -> np.ndarray:
        """
        Generate a vector of the new link state given a displacement.

        :param position: given displacement
        :return vector of new displacement state
        """
        vector = self.vector
        vector[2] += position
        return vector

    @property
    def kinematic_pair(self) -> KinematicPair:
        """
        Kinematic pair used.

        :return: kinematic pair
        """
        return KinematicPair.REVOLUTE
