"""Test."""
import numpy as np
from pytest import raises

from pybotics.errors import PyboticsError
from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.link import MDHLink, RevoluteMDHLink


def test_init() -> None:
    """Test."""
    # test error
    with raises(PyboticsError):
        MDHKinematicChain(np.eye(5))  # type: ignore

    # test sequence of links
    MDHKinematicChain([RevoluteMDHLink()])


def test_num_parameters() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    assert kc.num_parameters == MDHLink._size


def test_vector() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    np.testing.assert_allclose(kc.vector, link.vector)# type: ignore


def test_repr() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    repr(kc)


def test_to_json() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    kc.to_json()


def test_links_setter() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    kc.links = [link]


def test_ndof() -> None:
    """Test."""
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    assert kc.ndof == 1
