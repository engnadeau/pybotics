"""Test."""
import numpy as np
from pytest import raises

from pybotics.errors import PyboticsError
from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.link import MDHLink, RevoluteMDHLink


def test_init():
    """Test."""
    # test error
    with raises(PyboticsError):
        MDHKinematicChain(np.eye(5))

    # test sequence of links
    MDHKinematicChain([RevoluteMDHLink()])


def test_num_parameters():
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    assert kc.num_parameters == MDHLink._size


def test_vector():
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    np.testing.assert_allclose(kc.vector, link.vector)


def test_to_dict():
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    kc.to_dict()


def test_repr():
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    repr(kc)
