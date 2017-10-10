import numpy as np
from pytest import fixture

from pybotics.revolute_mdh_link import RevoluteMDHLink

LINK_PARAMETERS = [0, 10, np.pi / 2, 30]


@fixture(name='link')
def link_fixture():
    return RevoluteMDHLink(*LINK_PARAMETERS)


def test_vector(link):
    assert len(link.vector) == len(LINK_PARAMETERS)
    np.testing.assert_allclose(link.vector, LINK_PARAMETERS)


def test_displace(link):
    new_link_parameters = LINK_PARAMETERS.copy()
    new_link_parameters[2] -= np.pi / 2
    np.testing.assert_allclose(link.displace(-np.pi / 2), new_link_parameters)
