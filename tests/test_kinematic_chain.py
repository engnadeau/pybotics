import numpy as np
from pytest import raises

from pybotics.errors import PyboticsError
from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.link import RevoluteMDHLink, MDHLink


def test_init():
    # test error
    with raises(PyboticsError):
        MDHKinematicChain(np.eye(5))

    # test sequence of links
    MDHKinematicChain([RevoluteMDHLink()])


def test_num_parameters():
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    assert kc.num_parameters == MDHLink._size


def test_vector():
    link = RevoluteMDHLink()
    kc = MDHKinematicChain([link])
    np.testing.assert_allclose(kc.vector, link.vector)
