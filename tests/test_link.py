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
