import numpy as np
from pytest import fixture

from pybotics.tool import Tool


@fixture(name='tool')
def tool_fixture():
    return Tool()


def test_tcp_position(tool):
    values = [1.1, 2.2, 3.3]
    tool.tcp_position = values
    np.testing.assert_allclose(values, tool.matrix[:-1, -1])
