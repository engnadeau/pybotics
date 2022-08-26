"""Test robot."""
from pathlib import Path

import hypothesis
import numpy as np
import numpy.typing as npt
from hypothesis import given
from hypothesis.extra.numpy import arrays
from hypothesis.strategies import floats
from pytest import fixture, raises

from pybotics.errors import PyboticsError
from pybotics.kinematic_chain import MDHKinematicChain
from pybotics.predefined_models import ur10
from pybotics.robot import Robot


@fixture(scope="session")
def planar_robot() -> Robot:
    """Generate planar robot."""
    return Robot(
        MDHKinematicChain.from_parameters(
            np.array([[0, 0, 0, 0], [0, 10, 0, 0], [0, 20, 0, 0]])
        )
    )


def test_fk() -> None:
    """
    Test robot.

    :param robot:
    :return:
    """
    # load test data
    data_path = (
        Path(__file__).parent / "resources"
    ).resolve() / "ur10-joints-poses.csv"
    data = np.loadtxt(str(data_path), delimiter=",")  # type: ignore

    # load robot
    robot = Robot.from_parameters(ur10())

    # test
    for d in data:
        n = robot.ndof
        joints = np.deg2rad(d[:n])
        desired_pose = d[n:].reshape((4, 4))

        atol = 1e-3

        # test with position argument
        actual_pose = robot.fk(q=joints)
        np.testing.assert_allclose(actual_pose, desired_pose, atol=atol)  # type: ignore

        # test with internal position attribute
        robot.joints = joints
        actual_pose = robot.fk()
        np.testing.assert_allclose(actual_pose, desired_pose, atol=atol)  # type: ignore


def test_home_position() -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())
    x = np.ones(len(robot))
    robot.home_position = x
    np.testing.assert_allclose(robot.home_position, x)  # type: ignore


def test_joint_limits() -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())

    # test setter
    robot.joint_limits = robot.joint_limits.copy()

    # test errors
    with raises(PyboticsError):
        robot.joint_limits = np.zeros(1)

    with raises(PyboticsError):
        robot.joints = robot.joint_limits.copy()[1] + 10


def test_compute_joint_torques(planar_robot: Robot) -> None:
    """
    Test.

    From EXAMPLE 5.7 of
    Craig, John J. Introduction to robotics: mechanics and control.
    Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
    :return:
    """
    # set test force and angles
    force = np.array([-100, -200, 0])
    moment = np.zeros(3)
    wrench = force + moment  # type: npt.NDArray[np.float64]
    joint_angles = np.deg2rad([30, 60, 0])

    # get link lengths
    # FIXME: "Link" has no attribute "a"
    # TODO: review abstract inheritance
    lengths = [
        planar_robot.kinematic_chain.links[1].a,  # type: ignore
        planar_robot.kinematic_chain.links[2].a,  # type: ignore
    ]

    # calculate expected torques
    expected_torques = [
        lengths[0] * np.sin(joint_angles[1]) * force[0]
        + (lengths[1] + lengths[0] * np.cos(joint_angles[1])) * force[1],
        lengths[1] * force[1],
        0,
    ]

    # test
    actual_torques = planar_robot.compute_joint_torques(q=joint_angles, wrench=wrench)
    np.testing.assert_allclose(actual_torques, expected_torques)  # type: ignore

    planar_robot.joints = joint_angles
    actual_torques = planar_robot.compute_joint_torques(wrench=wrench)
    np.testing.assert_allclose(actual_torques, expected_torques)  # type: ignore


@given(
    q=arrays(
        shape=(3,),
        dtype=float,
        elements=floats(
            max_value=1e9, min_value=-1e9, allow_nan=False, allow_infinity=False
        ),
    )
)
def test_jacobian_world(q: npt.NDArray[np.float64], planar_robot: Robot) -> None:
    """Test."""
    # get link lengths
    # FIXME: "Link" has no attribute "a"
    # TODO: review abstract inheritance
    lengths = [
        planar_robot.kinematic_chain.links[1].a,  # type: ignore
        planar_robot.kinematic_chain.links[2].a,  # type: ignore
    ]

    # example from Craig has last joint set to 0
    q[-1] = 0

    s0 = np.sin(q[0])
    c0 = np.cos(q[0])

    s01 = np.sin(q[0] + q[1])
    c01 = np.cos(q[0] + q[1])

    expected = np.zeros((6, 3))
    expected[0, 0] = -lengths[0] * s0 - lengths[1] * s01
    expected[0, 1] = -lengths[1] * s01
    expected[1, 0] = lengths[0] * c0 + lengths[1] * c01
    expected[1, 1] = lengths[1] * c01
    expected[-1, :] = 1

    actual = planar_robot.jacobian_world(q)
    np.testing.assert_allclose(actual, expected, atol=1e-3)  # type: ignore


@given(
    q=arrays(
        shape=(3,), dtype=float, elements=floats(allow_nan=False, allow_infinity=False)
    )
)
def test_jacobian_flange(q: npt.NDArray[np.float64], planar_robot: Robot) -> None:
    """Test."""
    # get link lengths
    # FIXME: "Link" has no attribute "a"
    # TODO: review abstract inheritance
    lengths = [
        planar_robot.kinematic_chain.links[1].a,  # type: ignore
        planar_robot.kinematic_chain.links[2].a,  # type: ignore
    ]

    # example from Craig has last joint set to 0
    q[-1] = 0

    s1 = np.sin(q[1])
    c1 = np.cos(q[1])

    expected = np.zeros((6, 3))
    expected[0, 0] = lengths[0] * s1
    expected[1, 0] = lengths[0] * c1 + lengths[1]
    expected[1, 1] = lengths[1]
    expected[-1, :] = 1

    actual = planar_robot.jacobian_flange(q)
    np.testing.assert_allclose(actual, expected, atol=1e-6)  # type: ignore


@given(
    q=arrays(
        shape=(len(ur10()),),
        dtype=float,
        elements=floats(
            allow_nan=False, allow_infinity=False, max_value=np.pi, min_value=-np.pi
        ),
    ),
    q_offset=arrays(
        shape=(len(ur10()),),
        dtype=float,
        elements=floats(
            allow_nan=False,
            allow_infinity=False,
            max_value=np.deg2rad(1),
            min_value=np.deg2rad(-1),
        ),
    ),
)
@hypothesis.settings(deadline=None)
def test_ik(q: npt.NDArray[np.float64], q_offset: npt.NDArray[np.float64]) -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())
    pose = robot.fk(q)

    # IK is hard to solve without a decent seed
    q_actual = robot.ik(pose, q=robot.clamp_joints(q + q_offset))

    if q_actual is None:
        # ik couldn't be solved
        # don't fail test
        return

    actual_pose = robot.fk(q_actual)

    # test the matrix with lower accuracy
    # rotation components are hard to achieve when x0 isn't good
    np.testing.assert_allclose(actual_pose, pose, atol=1)  # type: ignore

    # test the position with higher accuracy
    desired_position = pose[:-1, -1]
    actual_position = actual_pose[:-1, -1]
    np.testing.assert_allclose(  # type: ignore
        actual_position, desired_position, atol=1e-1
    )


def test_random_joints() -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())
    robot.random_joints()
    robot.random_joints(in_place=True)


def test_to_json() -> None:
    """Test."""
    robot = Robot.from_parameters(ur10())
    robot.to_json()
