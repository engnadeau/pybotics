"""Test tool."""
import numpy as np
from pytest import raises

from pybotics.constants import POSITION_VECTOR_LENGTH
from pybotics.errors import SequenceLengthError


def test_cg(tool):
    new_cg = [9, 9, 9]
    tool.cg = new_cg
    np.testing.assert_allclose(tool.cg, new_cg)

    with raises(SequenceLengthError):
        tool.cg = np.ones(POSITION_VECTOR_LENGTH * 2)


def test_mass(tool):
    m = tool.mass
    tool.mass = m
