"""Test."""
import numpy as np

from pybotics.link import PrismaticMDHLink, RevoluteMDHLink


def test_len():
    """Test."""
    assert len(RevoluteMDHLink()) == 4


def test_displace():
    """Test."""
    link = PrismaticMDHLink()
    np.testing.assert_allclose(link.displace(), link.vector)


def test_to_dict():
    """Test dict function."""
    np.testing.assert_allclose(list(RevoluteMDHLink().to_dict().values()), 0)


def test_repr():
    """Test."""
    link = RevoluteMDHLink()
    repr(link)
