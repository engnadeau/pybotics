import hypothesis.strategies as st
import numpy as np
from hypothesis import given
from hypothesis.extra.numpy import arrays

from pybotics.optimization import compute_absolute_errors
from pybotics.predefined_models import UR10


@given(
    q=arrays(
        shape=(len(UR10()),),
        dtype=float,
        elements=st.floats(allow_nan=False,
                           allow_infinity=False)
    )
)
def test_compute_absolute_errors(q: np.ndarray):
    robot = UR10()
    pose = robot.fk(q)
    p = pose[:-1, -1]

    # test 1D input
    actual_error = compute_absolute_errors(qs=q, positions=p, robot=robot)
    np.testing.assert_allclose(actual_error, 0)

    # test 2D input
    actual_error = compute_absolute_errors(qs=np.tile(q, (10, 1)),
                                           positions=np.tile(p, (10, 1)),
                                           robot=robot)
    np.testing.assert_allclose(actual_error, 0)
