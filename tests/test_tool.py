"""Test."""
import numpy as np

from pybotics.geometry import matrix_2_vector
from pybotics.tool import Tool


def test_tool():
    """Test."""
    tool = Tool()

    cg = [1, 2, 3]
    tool.cg = cg
    np.testing.assert_allclose(tool.cg, cg)

    p = [1, 2, 3]
    tool.position = p
    np.testing.assert_allclose(tool.position, p)
    np.testing.assert_allclose(tool.vector, matrix_2_vector(tool.matrix))
