"""Test."""
import numpy as np

from pybotics.link import PrismaticMDHLink, RevoluteMDHLink


def test_len() -> None:
    """Test."""
    assert len(RevoluteMDHLink()) == 4


def test_displace() -> None:
    """Test."""
    link = PrismaticMDHLink()
    np.testing.assert_allclose(link.displace(), link.vector)  # type: ignore


def test_repr() -> None:
    """Test."""
    link = RevoluteMDHLink()
    repr(link)


def test_json() -> None:
    """Test."""
    link = RevoluteMDHLink()
    link.to_json()
