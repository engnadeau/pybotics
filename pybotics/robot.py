"""Robot module."""
from typing import Any, Optional, Sequence, Sized, Union

import attr
import numpy as np  # type: ignore
import scipy.optimize  # type: ignore

from pybotics.errors import PyboticsError
from pybotics.json_encoder import JSONEncoder
from pybotics.kinematic_chain import KinematicChain, MDHKinematicChain
from pybotics.tool import Tool


def _ndof_zeros_factory(robot: Any) -> np.ndarray:
    return np.zeros(len(robot.kinematic_chain))


def _joint_limits_factory(robot: Any) -> np.ndarray:
    return np.repeat((-np.pi, np.pi), len(robot.kinematic_chain)).reshape((2, -1))


@attr.s
class Robot(Sized):
    """Robot manipulator class."""

    kinematic_chain = attr.ib(type=KinematicChain)
    tool = attr.ib(factory=lambda: Tool(), type=Tool)
    world_frame = attr.ib(factory=lambda: np.eye(4), type=np.ndarray)  # type: ignore
    random_state = attr.ib(
        factory=lambda: np.random.RandomState(),  # type: ignore
        type=np.random.RandomState,
    )
    home_position = attr.ib(
        default=attr.Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=np.ndarray,
    )
    _joints = attr.ib(
        default=attr.Factory(factory=_ndof_zeros_factory, takes_self=True),
        type=np.ndarray,
    )
    _joint_limits = attr.ib(
        default=attr.Factory(factory=_joint_limits_factory, takes_self=True),
        type=np.ndarray,
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

    def fk(self, q: Optional[Sequence[float]] = None) -> np.ndarray:
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
            pose = np.dot(pose, t)

        return pose

    def ik(
        self, pose: np.ndarray, q: Optional[Sequence[float]] = None
    ) -> Optional[np.ndarray]:
        """Solve the inverse kinematics."""
        x0 = self.joints if q is None else q
        result = scipy.optimize.least_squares(
            fun=_ik_cost_function, x0=x0, bounds=self.joint_limits, args=(pose, self)
        )  # type: scipy.optimize.OptimizeResult

        if result.success:  # pragma: no cover
            actual_pose = self.fk(result.x)
            if np.allclose(actual_pose, pose, atol=1e-3):
                return result.x
        return None

    @property
    def ndof(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self)

    @property
    def joints(self) -> Union[Sequence[float], np.ndarray]:
        """
        Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._joints

    @joints.setter
    def joints(self, value: np.ndarray) -> None:
        """Set joints."""
        if np.any(value < self.joint_limits[0]) or np.any(value > self.joint_limits[1]):
            raise PyboticsError("Joint limits exceeded.")
        self._joints = value

    @property
    def joint_limits(self) -> np.ndarray:
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._joint_limits

    @joint_limits.setter
    def joint_limits(self, value: np.ndarray) -> None:
        """Set joint limits."""
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise PyboticsError(f"position_limits must have shape=(2,{len(self)})")
        self._joint_limits = value

    def jacobian_world(self, q: Optional[Sequence[float]] = None) -> np.ndarray:
        """Calculate the Jacobian wrt the world frame."""
        q = self.joints if q is None else q
        j_fl = self.jacobian_flange(q)
        pose = self.fk(q)
        rotation = pose[:3, :3]
        j_tr = np.zeros((6, 6), dtype=float)
        j_tr[:3, :3] = rotation
        j_tr[3:, 3:] = rotation
        j_w = np.dot(j_tr, j_fl)

        return j_w

    def jacobian_flange(self, q: Optional[Sequence[float]] = None) -> np.ndarray:
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
            current_transform = np.dot(current_link_transform, current_transform)

        return jacobian_flange

    def compute_joint_torques(
        self, wrench: Sequence[float], q: Optional[Sequence[float]] = None
    ) -> np.ndarray:
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
            force = np.dot(rotation, force)

            # calculate moment applied to current link
            q = transform[:3, -1]
            moment = np.dot(rotation, moment) + np.cross(q, force)

            # append z-component as joint torque
            joint_torques.append(moment[-1])

        # reverse torques into correct order
        return np.array(list(reversed(joint_torques)), dtype=float)

    def clamp_joints(self, q: Sequence[float]) -> Optional[np.ndarray]:
        """Limit joints to joint limits."""
        return np.clip(q, self.joint_limits[0], self.joint_limits[1])

    def random_joints(self, in_place: bool = False) -> Optional[np.ndarray]:
        """Generate random joints within limits."""
        q = self.random_state.uniform(
            low=self.joint_limits[0], high=self.joint_limits[1]
        )

        if in_place:
            self.joints = q
            return None
        else:
            return q

    @classmethod
    def from_parameters(cls, parameters: Sequence[float]) -> Sized:
        """Construct Robot from Kinematic Chain parameters."""
        # FIXME: assumes MDH revolute robot
        kc = MDHKinematicChain.from_parameters(parameters)
        return cls(kinematic_chain=kc)


def _ik_cost_function(q: np.ndarray, pose: np.ndarray, robot: Robot) -> np.ndarray:
    actual_pose = robot.fk(q)
    diff = np.abs(actual_pose - pose)
    return diff.ravel()
