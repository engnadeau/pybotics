from pytest import raises
import numpy as np
from pybotics import Tool
from pybotics.errors import PyboticsError
from pybotics.geometry import matrix_2_vector


def test_tool():
    tool = Tool()

    cg = [1, 2, 3]
    tool.cg = cg
    np.testing.assert_allclose(tool.cg, cg)

    with raises(PyboticsError):
        tool.cg = [1, 2, 3, 4]

    with raises(PyboticsError):
        tool.matrix = np.eye(5)

    p = [1, 2, 3]
    tool.position = p
    np.testing.assert_allclose(tool.position, p)

    np.testing.assert_allclose(tool.vector, matrix_2_vector(tool.matrix))
