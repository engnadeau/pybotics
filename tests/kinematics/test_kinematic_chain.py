from pytest import fixture, raises
import numpy as np

from pybotics.kinematics.convention import Convention
from pybotics.kinematics.dh_link import DHLink
from pybotics.kinematics.kinematic_chain import KinematicChain
from pybotics.kinematics.revolute_mdh_link import RevoluteMDHLink

# three link planar manipulator
MDH_ARRAY = np.array([
    [0, 0, 0, 0],
    [0, 10, 0, 0],
    [0, 20, 0, 0]
])

LINKS = [
    RevoluteMDHLink(*MDH_ARRAY[0]),
    RevoluteMDHLink(*MDH_ARRAY[1]),
    RevoluteMDHLink(*MDH_ARRAY[2])
]


@fixture(name='kc')
def kinematic_chain_fixture():
    return KinematicChain(LINKS)


def test_links(kc):
    # validate getter
    assert kc._links is kc.links

    # validate setter
    new_links = [RevoluteMDHLink(0, 0, 0, 0)]
    kc.links = new_links
    assert kc._links is kc.links
    assert kc._links is new_links
    assert kc._links is not LINKS

    # validate setter error checking
    new_links = [
        RevoluteMDHLink(0, 0, 0, 0),
        DHLink(0, 0, 0, 0)
    ]
    with raises(ValueError):
        kc.links = new_links


def test_transforms(kc):
    transforms = kc.transforms()

    # validate transforms
    assert len(transforms) == len(LINKS)
    for t, l in zip(transforms, LINKS):
        assert t[0, -1] == l.a


def test_from_array(kc):
    new_kc = KinematicChain.from_array(MDH_ARRAY, Convention.MDH)

    # TODO: implement __eq__() for kc
    assert len(new_kc.links) == len(kc.links)
    for new_link, link in zip(new_kc.links, kc.links):
        np.testing.assert_allclose(new_link.vector(), link.vector())


def test_from_revolute_mdh(kc):
    new_kc = KinematicChain.from_revolute_mdh(MDH_ARRAY)

    # TODO: implement __eq__() for kc
    assert len(new_kc.links) == len(kc.links)
    for new_link, link in zip(new_kc.links, kc.links):
        np.testing.assert_allclose(new_link.vector(), link.vector())


def test_num_dof(kc):
    assert kc.num_dof() == MDH_ARRAY.shape[0]


def test_validate_link_conventions():
    # validate positive case
    links = [
        DHLink(0, 0, 0, 0),
        DHLink(0, 0, 0, 0)
    ]
    KinematicChain.validate_link_conventions(links)

    # validate negative case
    links = [
        RevoluteMDHLink(0, 0, 0, 0),
        DHLink(0, 0, 0, 0)
    ]
    with raises(ValueError):
        KinematicChain.validate_link_conventions(links)


def test_convention():
    links = [
        DHLink(0, 0, 0, 0),
        DHLink(0, 0, 0, 0)
    ]
    kc = KinematicChain(links)
    assert kc.convention() is Convention.DH

    links = [
        RevoluteMDHLink(0, 0, 0, 0),
        RevoluteMDHLink(0, 0, 0, 0)
    ]
    kc = KinematicChain(links)
    assert kc.convention() is Convention.MDH
