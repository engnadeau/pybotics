"""Test MDH link."""
from pybotics.mdh_link import MDHLink


def test_abstract_methods():
    link = MDHLink(1, 2, 3, 4)
    link.displace(1)
    _ = link.kinematic_pair
