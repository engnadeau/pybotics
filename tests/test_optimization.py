"""Test."""
import hypothesis.strategies as st
import numpy as np
import scipy.optimize
from hypothesis import given
from hypothesis.extra.numpy import arrays
from pytest import raises

from pybotics.constants import TRANSFORM_VECTOR_LENGTH
from pybotics.errors import PyboticsError
from pybotics.optimization import (
    OptimizationHandler,
    compute_absolute_errors,
    optimize_accuracy,
)
from pybotics.predefined_models import UR10


@given(
    q=arrays(
        shape=(UR10.kinematic_chain.ndof,),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    )
)
def test_compute_absolute_errors(q: np.ndarray):
    """Test."""
    robot = UR10()
    pose = robot.fk(q)
    p = pose[:-1, -1]

    # test 1D input
    actual_error = compute_absolute_errors(qs=q, positions=p, robot=robot)
    np.testing.assert_allclose(actual_error, 0)

    # test 2D input
    actual_error = compute_absolute_errors(
        qs=np.tile(q, (10, 1)), positions=np.tile(p, (10, 1)), robot=robot
    )
    np.testing.assert_allclose(actual_error, 0)


def test_optimization():
    """Test."""
    # init robot model and error wrt nominal
    actual_robot = UR10()
    actual_robot.tool.position = [0.1, 0, 0]
    actual_robot.kinematic_chain.links[0].a += 0.1

    # calculate fk
    qs = np.tile(
        np.linspace(start=-np.pi, stop=np.pi, num=100), (UR10.kinematic_chain.ndof, 1)
    ).transpose()

    poses = np.array(list(map(actual_robot.fk, qs)))
    positions = poses[:, :-1, -1]

    # init handler
    handler = OptimizationHandler(robot=UR10())
    handler.kinematic_chain_mask[1] = True
    handler.tool_mask[0] = True

    # run optimization
    result = scipy.optimize.least_squares(
        fun=optimize_accuracy,
        x0=handler.generate_optimization_vector(),
        args=(handler, qs, positions),
        verbose=2,
    )  # type: scipy.optimize.OptimizeResult

    # validate
    atol = 1e-2
    np.testing.assert_allclose(
        actual=result.x, desired=handler.generate_optimization_vector(), atol=atol
    )
    np.testing.assert_allclose(
        actual=handler.robot.kinematic_chain.vector,
        desired=actual_robot.kinematic_chain.vector,
        atol=atol,
    )
    np.testing.assert_allclose(
        actual=handler.robot.tool.vector, desired=actual_robot.tool.vector, atol=atol
    )
    np.testing.assert_allclose(
        actual=handler.robot.world_frame, desired=actual_robot.world_frame, atol=atol
    )


def test_handler_validate_transform_mask():
    """Test."""
    # test predesigned mask sequence
    OptimizationHandler(robot=UR10(), tool_mask=[False] * TRANSFORM_VECTOR_LENGTH)

    # test error
    with raises(PyboticsError):
        OptimizationHandler(robot=UR10(), kinematic_chain_mask=[False])
