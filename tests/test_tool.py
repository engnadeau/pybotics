import numpy as np
import pytest

from pybotics import Tool
from pybotics.exceptions import PybotException


@pytest.fixture(name='tool')
def tool_fixture():
    return Tool()


def test_init(tool):
    assert isinstance(tool.tcp, np.ndarray)
    assert isinstance(tool.mass, (float, int))
    assert isinstance(tool.cg, np.ndarray)


def test_xyz(tool):
    values = [1.1, 2.2, 3.3]
    tool.tcp_xyz(values)
    np.testing.assert_allclose(values, tool.tcp[:-1, -1])

    with pytest.raises(PybotException):
        tool.tcp_xyz(values + values)
