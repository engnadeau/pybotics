"""Kinematic chain test."""
import numpy as np
from pytest import raises

from pybotics import KinematicPair
from pybotics.constants import TRANSFORM_MATRIX_SHAPE
from pybotics.errors import LinkConventionError, KinematicPairError, \
    SequenceLengthError, LinkSequenceError
from pybotics.kinematic_chain import KinematicChain
from pybotics.link import Link
from pybotics.revolute_mdh_link import RevoluteMDHLink


def test_len(planar_kc):
    """
    Test KC.

    :param planar_kc:
    :return:
    """
    assert len(planar_kc) == 3


def test_optimization(planar_kc):
    """
    Test KC.

    :param planar_kc:
    :return:
    """
    masked_index = 1

    # test mask
    planar_kc.optimization_mask = True
    # PyCharm thinks optimization_mask is just a bool...
    # noinspection PyTypeChecker
    assert all(planar_kc.optimization_mask)

    mask = planar_kc.optimization_mask
    # PyCharm thinks optimization_mask is just a bool...
    # noinspection PyUnresolvedReferences
    mask[masked_index] = False
    planar_kc.optimization_mask = mask
    # PyCharm thinks optimization_mask is just a bool...
    # noinspection PyTypeChecker
    assert sum(planar_kc.optimization_mask) == len(
        planar_kc.optimization_mask) - 1

    # test optimization vector
    masked_element = planar_kc.vector[masked_index]
    new_optimization_vector = planar_kc.optimization_vector * 2
    planar_kc.apply_optimization_vector(new_optimization_vector)

    assert planar_kc.vector[masked_index] == masked_element

    leftover_vector = [e for i, e in enumerate(planar_kc.vector) if
                       i != masked_index]
    np.testing.assert_allclose(leftover_vector, new_optimization_vector)


def test_array_2_links():
    """
    Test KC.

    :return:
    """
    KinematicChain.array_2_links(np.ones((3, 4)),
                                 kinematic_pairs=3 * [KinematicPair.REVOLUTE])

    with raises(LinkConventionError):
        # noinspection PyTypeChecker
        KinematicChain.array_2_links(np.ones(4), None)
    with raises(KinematicPairError):
        # noinspection PyTypeChecker
        KinematicChain.array_2_links(np.ones(4), kinematic_pairs=[123])
    with raises(SequenceLengthError):
        KinematicChain.array_2_links(np.ones(TRANSFORM_MATRIX_SHAPE),
                                     kinematic_pairs=[KinematicPair.REVOLUTE])


def test_links():
    """
    Test KC.

    :return:
    """
    with raises(LinkSequenceError):
        links = [
            RevoluteMDHLink(1, 2, 3, 4),
            Link()
        ]
        KinematicChain(links)


def test_num_dof(planar_kc):
    """
    Test KC.

    :param planar_kc:
    :return:
    """
    assert len(planar_kc) == planar_kc.num_dof


def test_transforms(planar_kc):
    """
    Test KC.

    :param planar_kc:
    :return:
    """
    # test normal case
    transforms = planar_kc.transforms()
    assert len(transforms) == len(planar_kc)
    for tr, link in zip(transforms, planar_kc.links):
        np.testing.assert_almost_equal(tr[0, -1], link.vector[1])

    # test normal case
    planar_kc.transforms(np.ones(len(planar_kc)))

    # test validation
    with raises(SequenceLengthError):
        planar_kc.transforms(np.ones(len(planar_kc) * 2))


def test_from_array():
    """
    Test KC.

    :return:
    """
    KinematicChain.from_array(np.ones(4))
