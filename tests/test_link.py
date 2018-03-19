import numpy as np

from pybotics.link import RevoluteMDHLink, PrismaticMDHLink


def test_len():
    assert len(RevoluteMDHLink()) == 4


def test_displace():
    link = PrismaticMDHLink()
    np.testing.assert_allclose(link.displace(), link.vector)
