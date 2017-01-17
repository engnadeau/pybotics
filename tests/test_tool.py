from pybotics import Tool
import numpy as np
import pytest

np.set_printoptions(suppress=True)


def test_init():
    tool = Tool()
    assert isinstance(tool.tcp, np.ndarray)
    assert isinstance(tool.mass, (float, int))
    assert isinstance(tool.cg, np.ndarray)
