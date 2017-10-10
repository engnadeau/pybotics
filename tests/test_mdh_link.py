import numpy as np
from pytest import fixture, warns

from pybotics.mdh_link import MDHLink

np.set_printoptions(suppress=True)

LINK_PARAMETERS = [0, 10, np.pi / 2, 30]


@fixture(name='link')
def link_fixture():
    return MDHLink(*LINK_PARAMETERS)


def test_vector(link):
    assert len(link.vector) == len(LINK_PARAMETERS)
    np.testing.assert_allclose(link.vector, LINK_PARAMETERS)


def test_transform(link):
    desired_transform = np.array([
        0, -1, 0, 10,
        1, 0, 0, 0,
        0, 0, 1, 30,
        0, 0, 0, 1
    ]).reshape((4, 4))
    with warns(UserWarning):
        np.testing.assert_allclose(desired=desired_transform, actual=link.transform(), atol=1e-7)
