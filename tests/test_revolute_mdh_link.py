"""Test revolute MDH link."""
from pybotics.kinematic_pair import KinematicPair
from pybotics.revolute_mdh_link import RevoluteMDHLink


def test_kinematic_pair():
    """
    Test link.

    :return:
    """
    link = RevoluteMDHLink(0, 0, 0, 0)
    assert link.kinematic_pair is KinematicPair.REVOLUTE
