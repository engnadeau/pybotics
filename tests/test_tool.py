"""Test."""
import numpy as np

from pybotics.geometry import matrix_2_vector
from pybotics.tool import Tool


def test_tool() -> None:
    """Test."""
    tool = Tool()

    cg = np.array([1, 2, 3])
    tool.cg = cg
    np.testing.assert_allclose(tool.cg, cg)  # type: ignore

    p = np.array([1, 2, 3])
    tool.position = p
    np.testing.assert_allclose(tool.position, p)  # type: ignore
    np.testing.assert_allclose(tool.vector, matrix_2_vector(tool.matrix))  # type: ignore
