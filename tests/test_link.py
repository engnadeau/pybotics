"""Test link."""
import pybotics.conventions as conv
import pybotics.link as lk


def test_abstract_methods():
    """
    Test link.

    :return:
    """
    link = lk.Link()
    link.displace(1)
    assert link.convention is None
    assert link.kinematic_pair is None
    link.transform()
    assert link.vector is None


def test_kinematic_pair():
    """
    Test link.

    :return:
    """
    link = lk.RevoluteMDHLink(0, 0, 0, 0)
    assert link.kinematic_pair is conv.KinematicPair.REVOLUTE
