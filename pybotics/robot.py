"""Robot module."""
from json import JSONEncoder
from typing import Optional, Sequence, Sized, Any

import numpy as np  # type: ignore
import scipy.optimize

from pybotics.constants import ROTATION_VECTOR_LENGTH, TRANSFORM_MATRIX_SHAPE
from pybotics.errors import PyboticsError
from pybotics.kinematic_chain import KinematicChain
from pybotics.tool import Tool


class Robot(Sized):
    """Robot manipulator class."""

    def __init__(self,
                 kinematic_chain: KinematicChain,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[np.ndarray] = None
                 ) -> None:
        """
        Construct a robot instance.

        :param kinematic_chain: kinematic chain of the manipulator
        :param tool: attached tool
        :param world_frame: transform of robot base with respect to the world
        """
        self._home_position = np.zeros(len(kinematic_chain))
        self._joints = np.zeros(len(kinematic_chain))
        self._joint_limits = np.repeat((-np.pi, np.pi),
                                       len(kinematic_chain)
                                       ).reshape((2, -1))

        self.kinematic_chain = kinematic_chain
        self.world_frame = np.eye(TRANSFORM_MATRIX_SHAPE[0]) \
            if world_frame is None else world_frame
        self.tool = Tool() if tool is None else tool

    def __len__(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return len(self.kinematic_chain)

    def __repr__(self) -> str:
        """
        Get the debug representation of the robot model.

        :return:
        """
        return self.to_json()

    def to_json(self) -> str:
        encoder = RobotJSONEncoder(sort_keys=True)
        return encoder.encode(self)

    def __str__(self) -> str:
        """
        Get the string representation of the robot model.

        :return:
        """
        return self.__repr__()

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

    def ik(self,
           pose: np.ndarray,
           q: Optional[Sequence[float]] = None,
           ) -> Optional[np.ndarray]:

        x0 = self.joints if q is None else q
        result = scipy.optimize.least_squares(
            fun=_ik_cost_function,
            x0=x0,
            bounds=self.joint_limits,
            args=(pose, self),
        )  # type: scipy.optimize.OptimizeResult

        if result.success:
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
    def joints(self) -> np.ndarray:
        """
        Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._joints

    @joints.setter
    def joints(self, value: np.ndarray) -> None:
        # TODO: check if position is in limits
        self._joints = value

    @property
    def home_position(self) -> np.ndarray:
        """
        Get the robot configuration (e.g., joint positions for serial robot).

        :return: robot position
        """
        return self._home_position

    @home_position.setter
    def home_position(self, value: np.ndarray) -> None:
        # TODO: check if position is in limits
        self._home_position = value

    @property
    def joint_limits(self) -> np.ndarray:
        """
        Limits of the robot position (e.g., joint limits).

        :return: limits with shape (2,num_dof) where first row is upper limits
        """
        return self._joint_limits

    @joint_limits.setter
    def joint_limits(self, value: np.ndarray) -> None:
        if value.shape[0] != 2 or value.shape[1] != len(self):
            raise PyboticsError(
                'position_limits must have shape=(2,{})'.format(len(self)))
        self._joint_limits = value

    def jacobian_world(self,
                       q: Optional[Sequence[float]] = None) -> np.ndarray:
        q = self.joints if q is None else q
        j_fl = self.jacobian_flange(q)
        pose = self.fk(q)
        rotation = pose[:3, :3]
        j_tr = np.zeros(
            (ROTATION_VECTOR_LENGTH * 2, ROTATION_VECTOR_LENGTH * 2),
            dtype=float
        )
        j_tr[:ROTATION_VECTOR_LENGTH, :ROTATION_VECTOR_LENGTH] = \
            rotation
        j_tr[ROTATION_VECTOR_LENGTH:, ROTATION_VECTOR_LENGTH:] = \
            rotation
        j_w = np.dot(j_tr, j_fl)

        return j_w

    def jacobian_flange(self,
                        q: Optional[Sequence[float]] = None) -> np.ndarray:
        q = self.joints if q is None else q

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.ndof))
        current_transform = self.tool.matrix.copy()

        for i in reversed(range(self.ndof)):
            d = np.array([
                -current_transform[0, 0] * current_transform[1, 3] +
                current_transform[1, 0] * current_transform[0, 3],
                - current_transform[0, 1] * current_transform[1, 3] +
                current_transform[1, 1] * current_transform[0, 3],
                - current_transform[0, 2] * current_transform[1, 3] +
                current_transform[1, 2] * current_transform[0, 3],
            ])
            delta = current_transform[2, 0:3]

            jacobian_flange[:, i] = np.hstack((d, delta))

            current_link = self.kinematic_chain.links[i]
            p = q[i]  # type: ignore
            current_link_transform = current_link.transform(p)
            current_transform = np.dot(current_link_transform,
                                       current_transform)

        return jacobian_flange

    def compute_joint_torques(self,
                              wrench: Sequence[float],
                              q: Optional[Sequence[float]] = None,
                              ) -> np.ndarray:
        """
        Calculate the joint torques
        due to external force applied to the flange frame.
        Method from:
        5.9 STATIC FORCES IN MANIPULATORS
        Craig, John J. Introduction to robotics: mechanics and control.
        Vol. 3. Upper Saddle River: Pearson Prentice Hall, 2005.
        :param wrench:
        :param q:
        :return:
        """
        q = self.joints if q is None else q

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

    def clamp_joints(self, q: Sequence[float]) -> np.ndarray:
        return np.clip(q, self.joint_limits[0], self.joint_limits[1])


class RobotJSONEncoder(JSONEncoder):
    """Robot JSON Encoder class."""

    def default(self, o: Any) -> Any:  # pragma: no cover
        """
        Return serializable robot objects.

        :param o:
        :return:
        """
        # process custom instances
        if isinstance(o, np.ndarray):
            return o.tolist()

        if isinstance(o, np.generic):
            return str(o)

        try:
            o = o.__dict__
        except AttributeError:
            pass
        else:
            return o

        # let the base class default method raise the TypeError
        # https://docs.python.org/3/library/json.html
        return JSONEncoder.default(self, o)


def _ik_cost_function(q: np.ndarray,
                      pose: np.ndarray,
                      robot: Robot,
                      ) -> np.ndarray:
    actual_pose = robot.fk(q)
    diff = np.abs(actual_pose - pose)
    return diff.ravel()
