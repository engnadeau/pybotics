"""Test tool."""
import numpy as np


def test_cg(tool):
    """
    Test tool.

    :param tool:
    :return:
    """
    new_cg = [9, 9, 9]
    tool.cg = new_cg
    np.testing.assert_allclose(tool.cg, new_cg)


def test_mass(tool):
    """
    Test tool.

    :param tool:
    :return:
    """
    m = tool.mass
    tool.mass = m
