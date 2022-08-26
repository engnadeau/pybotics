"""Robot module."""
from __future__ import annotations

import typing
from typing import Any, Optional, Sized

import attr
import numpy as np
import numpy.typing as npt
import scipy.optimize  # type: ignore

from pybotics.errors import PyboticsError
from pybotics.json_encoder import JSONEncoder
from pybotics.kinematic_chain import KinematicChain, MDHKinematicChain
from pybotics.tool import Tool


def _ndof_zeros_factory(robot: Any) -> npt.NDArray[np.float64]:
    return np.zeros(len(robot.kinematic_chain))


def _joint_limits_factory(robot: Any) -> npt.NDArray[np.float64]:
    return np.repeat((-np.pi, np.pi), len(robot.kinematic_chain)).reshape((2, -1))


@attr.s
class Robot(Sized):
    """Robot manipulator class."""

    kinematic_chain = attr.ib(type=KinematicChain)
    tool = attr.ib(factory=lambda: Tool(), type=Tool)
    world_frame = attr.ib(factory=lambda: np.eye(4), type=npt.NDArray[np.float64])
    random_state = attr.ib(
        factory=lambda: np.random.RandomState(),
        type=np.random.RandomState,
    )
    home_position = attr.ib(
        default=attr.Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=npt.NDArray[np.float64],
    )
    _joints = attr.ib(
        default=attr.Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=npt.NDArray[np.float64],
    )
    _joint_limits = attr.ib(
        default=attr.Factory(factory=_joint_limits_factory, takes_self=True),
        type=npt.NDArray[np.float64],
    )

    def __len__(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self.kinematic_chain)

    def to_json(self) -> str:
        """Encode robot model as JSON."""
        encoder = JSONEncoder(sort_keys=True)
        return encoder.encode(self)

    def fk(
        self, q: Optional[npt.NDArray[np.float64]] = None
    ) -> npt.NDArray[np.float64]:
        """
        Compute the forward kinematics of a given position.

        Uses the current position if None is given.
        :param q:
        :return: 4x4 transform matrix of the FK pose
        """
        # validate
        q = self.joints if q is None else q

        # gather transforms
        # noinspection PyListCreation
        transforms = []
        transforms.append(self.world_frame)
        transforms.extend(self.kinematic_chain.transforms(q))
        transforms.append(self.tool.matrix)

        # matrix multiply through transforms
        pose = np.eye(4, dtype=float)
        for t in transforms:
            pose = np.dot(pose, t)  # type: ignore

        return pose

    def ik(
        self, pose: npt.NDArray[np.float64], q: Optional[npt.NDArray[np.float64]] = None
    ) -> Optional[npt.NDArray[np.float64]]:
        """Solve the inverse kinematics."""
        # set initial value
        x0 = self.joints if q is None else q

        # solve optimization
        optimization_result = scipy.optimize.least_squares(
            fun=_ik_cost_function, x0=x0, bounds=self.joint_limits, args=(pose, self)
        )  # type: scipy.optimize.OptimizeResult

        # set return value
        result = None  # type: Optional[npt.NDArray[np.float64]]
        if optimization_result.success:  # pragma: no cover
            actual_pose = self.fk(optimization_result.x)
            if np.allclose(actual_pose, pose, atol=1e-3):
                result = optimization_result.x

        return result

    @property
    def ndof(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self)

    @property
    def joints(self) -> npt.NDArray[np.float64]:
        """
        Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._joints

    @joints.setter
    def joints(self, value: npt.NDArray[np.float64]) -> None:
        """Set joints."""
        if np.any(value < self.joint_limits[0]) or np.any(value > self.joint_limits[1]):
            raise PyboticsError("Joint limits exceeded.")
        self._joints = value

    @property
    def joint_limits(self) -> npt.NDArray[np.float64]:
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._joint_limits

    @joint_limits.setter
    def joint_limits(self, value: npt.NDArray[np.float64]) -> None:
        """Set joint limits."""
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise PyboticsError(f"position_limits must have shape=(2,{len(self)})")
        self._joint_limits = value

    def jacobian_world(
        self, q: Optional[npt.NDArray[np.float64]] = None
    ) -> npt.NDArray[np.float64]:
        """Calculate the Jacobian wrt the world frame."""
        q = self.joints if q is None else q
        j_fl = self.jacobian_flange(q)
        pose = self.fk(q)
        rotation = pose[:3, :3]
        j_tr = np.zeros((6, 6), dtype=float)
        j_tr[:3, :3] = rotation
        j_tr[3:, 3:] = rotation
        j_w = np.dot(j_tr, j_fl)  # type: ignore

        return typing.cast(npt.NDArray[np.float64], j_w)

    def jacobian_flange(
        self, q: Optional[npt.NDArray[np.float64]] = None
    ) -> npt.NDArray[np.float64]:
        """Calculate the Jacobian wrt the flange frame."""
        q = self.joints if q is None else q

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.ndof))
        current_transform = self.tool.matrix.copy()

        for i in reversed(range(self.ndof)):
            d = np.array(
                [
                    -current_transform[0, 0] * current_transform[1, 3]
                    + current_transform[1, 0] * current_transform[0, 3],
                    -current_transform[0, 1] * current_transform[1, 3]
                    + current_transform[1, 1] * current_transform[0, 3],
                    -current_transform[0, 2] * current_transform[1, 3]
                    + current_transform[1, 2] * current_transform[0, 3],
                ]
            )
            delta = current_transform[2, 0:3]

            jacobian_flange[:, i] = np.hstack((d, delta))

            current_link = self.kinematic_chain.links[i]
            p = q[i]
            current_link_transform = current_link.transform(p)
            current_transform = np.dot(  # type: ignore
                current_link_transform, current_transform
            )

        return jacobian_flange

    def compute_joint_torques(
        self,
        wrench: npt.NDArray[np.float64],
        q: Optional[npt.NDArray[np.float64]] = None,
    ) -> npt.NDArray[np.float64]:
        """
        Calculate the joint torques due to external flange force.

        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
        :param wrench:
        :param q:
        :return:
        """
        if q is None:
            q = self.joints

        # split wrench into components
        force = wrench[:3]
        moment = wrench[-3:]

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base
        # each iteration calculates for link i-1
        for i, p in reversed(list(enumerate(q))):  # pragma: no cover
            if i == 0:
                break

            # get current link transform
            transform = self.kinematic_chain.links[i].transform(p)

            # calculate force applied to current link
            rotation = transform[:3, :3]
            force = np.dot(rotation, force)  # type: ignore

            # calculate moment applied to current link
            q = transform[:3, -1]
            moment = np.dot(rotation, moment) + np.cross(q, force)  # type: ignore

            # append z-component as joint torque
            joint_torques.append(moment[-1])

        # reverse torques into correct order
        return np.array(list(reversed(joint_torques)), dtype=float)

    def clamp_joints(self, q: npt.NDArray[np.float64]) -> npt.NDArray[np.float64]:
        """Limit joints to joint limits."""
        result = np.clip(
            q, self.joint_limits[0], self.joint_limits[1]
        )  # type: npt.NDArray[np.float64]
        return result

    def random_joints(
        self, in_place: bool = False
    ) -> Optional[npt.NDArray[np.float64]]:
        """Generate random joints within limits."""
        q = self.random_state.uniform(
            low=self.joint_limits[0], high=self.joint_limits[1]
        )

        result = None
        if in_place:
            self.joints = q
        else:
            result = q

        return result

    @classmethod
    def from_parameters(cls, parameters: npt.NDArray[np.float64]) -> Robot:
        """Construct Robot from Kinematic Chain parameters."""
        # FIXME: assumes MDH revolute robot
        kc = MDHKinematicChain.from_parameters(parameters)
        return cls(kinematic_chain=kc)


def _ik_cost_function(
    q: npt.NDArray[np.float64], pose: npt.NDArray[np.float64], robot: Robot
) -> npt.NDArray[np.float64]:
    actual_pose = robot.fk(q)
    diff = np.abs(actual_pose - pose)
    result = diff.ravel()  # type: npt.NDArray[np.float64]
    return result
