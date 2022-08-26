"""Test.

isort:skip_file
"""
import hypothesis.strategies as st
import numpy as np
import numpy.typing as npt
import scipy.optimize  # type: ignore
from hypothesis import given
from hypothesis.extra.numpy import arrays
from pytest import raises

from pybotics.errors import PyboticsError
from pybotics.optimization import (
    OptimizationHandler,
    compute_absolute_error,
    compute_absolute_errors,
    optimize_accuracy,
    compute_relative_error,
    compute_relative_errors,
)
from pybotics.predefined_models import ur10
from pybotics.robot import Robot


@given(
    q=arrays(
        shape=(len(ur10()),),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    )
)
def test_compute_absolute_errors(q: npt.NDArray[np.float64]) -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())
    pose = robot.fk(q)
    position_vector = pose[:-1, -1]

    # test 1D input; i.e., only one pose
    actual_error = compute_absolute_error(q=q, position=position_vector, robot=robot)
    np.testing.assert_allclose(actual_error, 0)  # type: ignore

    # test 2D input; i.e., many poses
    num_repeats = 10
    actual_error = compute_absolute_errors(
        qs=np.tile(q, (num_repeats, 1)),  # type: ignore
        positions=np.tile(position_vector, (num_repeats, 1)),  # type: ignore
        robot=robot,
    )
    np.testing.assert_allclose(actual_error, 0)  # type: ignore


@given(
    q_a=arrays(
        shape=(len(ur10()),),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    ),
    q_b=arrays(
        shape=(len(ur10()),),
        dtype=float,
        elements=st.floats(allow_nan=False, allow_infinity=False),
    ),
)
def test_compute_relative_errors(
    q_a: npt.NDArray[np.float64], q_b: npt.NDArray[np.float64]
) -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())

    p_a = robot.fk(q_a)[:-1, -1]
    p_b = robot.fk(q_b)[:-1, -1]
    distance = np.linalg.norm(p_a - p_b)  # type: ignore

    # test 1D input; i.e., only one pose
    actual_error = compute_relative_error(
        q_a=q_a, q_b=q_b, distance=distance, robot=robot
    )
    np.testing.assert_allclose(actual_error, 0)  # type: ignore

    # test 2D input; i.e., many poses
    num_repeats = 10
    actual_error = compute_relative_errors(
        qs_a=np.tile(q_a, (num_repeats, 1)),  # type: ignore
        qs_b=np.tile(q_b, (num_repeats, 1)),  # type: ignore
        distances=np.tile(distance, (num_repeats, 1)),  # type: ignore
        robot=robot,
    )
    np.testing.assert_allclose(actual_error, 0)  # type: ignore


def test_optimization() -> None:
    """Test."""
    # init robot model and error wrt nominal
    actual_robot = Robot.from_parameters(ur10())
    actual_robot.tool.position = np.array([0.1, 0, 0])
    # FIXME: "Link" has no attribute "a"
    # TODO: review abstract inheritance
    actual_robot.kinematic_chain.links[0].a += 0.1  # type: ignore

    # calculate fk
    qs = np.tile(
        np.linspace(start=-np.pi, stop=np.pi, num=100), (len(ur10()), 1)
    ).transpose()  # type: ignore

    poses = np.array(list(map(actual_robot.fk, qs)))
    positions = poses[:, :-1, -1]

    # init handler
    handler = OptimizationHandler(robot=Robot.from_parameters(ur10()))
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
    )  # type: ignore
    np.testing.assert_allclose(
        actual=handler.robot.kinematic_chain.vector,
        desired=actual_robot.kinematic_chain.vector,
        atol=atol,
    )  # type: ignore
    np.testing.assert_allclose(
        actual=handler.robot.tool.vector, desired=actual_robot.tool.vector, atol=atol
    )  # type: ignore
    np.testing.assert_allclose(
        actual=handler.robot.world_frame, desired=actual_robot.world_frame, atol=atol
    )  # type: ignore


def test_handler_validate_transform_mask() -> None:
    """Test."""
    # test predesigned mask sequence
    OptimizationHandler(robot=Robot.from_parameters(ur10()), tool_mask=[False] * 6)

    # test error
    with raises(PyboticsError):
        OptimizationHandler(
            robot=Robot.from_parameters(ur10()), kinematic_chain_mask=[False]
        )
