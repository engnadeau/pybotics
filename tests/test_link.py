"""Test link."""
from pybotics.link import Link


def test_abstract_methods():
    link = Link()
    link.displace(1)
    _ = link.convention
    _ = link.kinematic_pair
    link.transform()
    _ = link.vector
