"""Test link."""
from pybotics.link import Link


def test_abstract_methods():
    """
    Test link.

    :return:
    """
    link = Link()
    link.displace(1)
    assert link.convention is None
    assert link.kinematic_pair is None
    link.transform()
    assert link.vector is None
