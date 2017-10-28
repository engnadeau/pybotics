"""Test MDH link."""
from pybotics.mdh_link import MDHLink


def test_abstract_methods():
    """
    Test link.

    :return:
    """
    link = MDHLink(1, 2, 3, 4)
    link.displace(1)
    assert link.kinematic_pair is None
