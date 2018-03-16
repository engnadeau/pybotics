"""Robot module."""
import logging
from json import JSONEncoder
from typing import Optional, Sequence, Sized, Any

import numpy as np  # type: ignore
import scipy.optimize

from pybotics.constants import ROTATION_VECTOR_LENGTH
from pybotics.errors import PyboticsError
from pybotics.frame import Frame
from pybotics.geometry import _matrix_2_euler_zyx, wrap_2_pi
from pybotics.kinematic_chain import KinematicChain
from pybotics.tool import Tool


class Robot(Sized):
    """Robot manipulator class."""

    def __init__(self, kinematic_chain: KinematicChain,
                 tool: Optional[Tool] = None,
                 world_frame: Optional[Frame] = None) -> None:
        """
        Construct a robot instance.

        :param kinematic_chain: kinematic chain of the manipulator
        :param tool: attached tool
        :param world_frame: transform of robot base with respect to the world
        """
        self._home_position = np.zeros(kinematic_chain.num_dof)
        self._joints = np.zeros(kinematic_chain.num_dof)
        self._joint_limits = np.repeat((-np.pi, np.pi),
                                       kinematic_chain.num_dof
                                       ).reshape((2, -1))

        self.world_frame = Frame() if world_frame is None else world_frame
        self.kinematic_chain = kinematic_chain
        self.tool = Tool() if tool is None else tool

    def __len__(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return self.kinematic_chain.num_dof

    def __repr__(self) -> str:
        """
        Get the debug representation of the robot model.

        :return:
        """
        encoder = RobotJSONEncoder(sort_keys=True, indent=4)
        return encoder.encode(self)

    def __str__(self) -> str:
        """
        Get the string representation of the robot model.

        :return:
        """
        return self.__repr__()

    def apply_optimization_vector(self, vector: np.ndarray) -> None:
        """
        Update the current instance with new optimization parameters.

        :param vector: new parameters to apply
        """
        split_vector = np.split(
            vector,
            np.cumsum(
                [len(self.world_frame.optimization_vector),
                 len(self.kinematic_chain.optimization_vector)]
            ))
        self.world_frame.apply_optimization_vector(split_vector[0])
        self.kinematic_chain.apply_optimization_vector(split_vector[1])
        self.tool.apply_optimization_vector(split_vector[2])

    def fk(self, q: Optional[Sequence[float]] = None) -> np.ndarray:
        """
        Compute the forward kinematics of a given position.

        Uses the current position if None is given.
        :param q:
        :return: 4x4 transform matrix of the FK pose
        """
        # validate
        if q is None:
            q = self.joints

        # gather transforms
        transforms = [self.world_frame.matrix]
        transforms.extend(self.kinematic_chain.transforms(q))
        transforms.append(self.tool.matrix)

        # matrix multiply through transforms
        pose = np.eye(4, dtype=float)
        for t in transforms:
            pose = np.dot(pose, t)

        return pose

    def ik_jacobian(self, pose: np.ndarray, q: Optional[Sequence[float]] = None,
                    alpha: float = 0.1, atol=1e-6, max_iter=1e3) -> Optional[
        np.ndarray]:

        # set seed joints
        q = self.joints if q is None else np.array(q)
        j = self.jacobian_world(q)
        det = np.linalg.det(j)
        cond = np.linalg.cond(j)
        if np.allclose(det, 0) or cond > 100:
            logging.getLogger(__name__).warning(
                'Initial joints are near singularity; '
                'IK solver may not converge')

        # convert pose to vector
        desired_vector = _matrix_2_euler_zyx(pose)

        # solve IK
        is_running = True
        num_iterations = 0
        last_error_norm = np.inf
        last_q = q
        while is_running:
            num_iterations += 1

            # get jacobian and pseudo inverse
            j = self.jacobian_world(q)
            j_pinv = np.linalg.pinv(j)

            # get current error
            current_pose = self.fk(q)
            current_vector = _matrix_2_euler_zyx(current_pose)
            error = desired_vector - current_vector
            error_norm = np.linalg.norm(error)

            # adjust alpha if needed
            if error_norm < last_error_norm:
                last_error_norm = error_norm
                alpha *= 2
            else:
                logging.debug('Error has diverged; '
                              'restoring last iteration; '
                              'reducing alpha')
                alpha /= 2
                q = last_q.copy()

            if np.allclose(error, 0, atol=atol):
                is_running = False
            elif num_iterations >= max_iter:
                logging.getLogger(__name__).warning(
                    'Maximum number of iterations reached')
                is_running = False
            else:
                # update joints
                dq = np.dot(j_pinv, error)
                last_q = q.copy()
                q += alpha * dq
                q = np.array([wrap_2_pi(x) for x in q])

        return q

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
            return result.x
        else:
            return None

    @property
    def num_dof(self) -> int:
        """
        Get the number of degrees of freedom.

        :return: number of degrees of freedom
        """
        return self.kinematic_chain.num_dof

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

    def jacobian_world(self, q: Optional[Sequence[float]] = None) -> np.ndarray:
        if q is None:
            q = self.joints

        jacobian_flange = self.jacobian_flange(q)
        pose = self.fk(q)

        rotation = pose[:3, :3]

        jacobian_transform = np.zeros(
            (ROTATION_VECTOR_LENGTH * 2, ROTATION_VECTOR_LENGTH * 2),
            dtype=float
        )

        jacobian_transform[:ROTATION_VECTOR_LENGTH, :ROTATION_VECTOR_LENGTH] = \
            rotation
        jacobian_transform[ROTATION_VECTOR_LENGTH:, ROTATION_VECTOR_LENGTH:] = \
            rotation
        jacobian_world = np.dot(jacobian_transform, jacobian_flange)

        return jacobian_world

    def jacobian_flange(self,
                        q: Optional[Sequence[float]] = None) -> np.ndarray:
        q = self.joints if q is None else q

        # init Cartesian jacobian (6-dof in space)
        jacobian_flange = np.zeros((6, self.num_dof))
        current_transform = self.tool.matrix.copy()

        for i in reversed(range(self.num_dof)):
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

    def calculate_joint_torques(self, q: Sequence[float],
                                wrench: Sequence[float]) -> np.ndarray:
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

        # split wrench into components
        force = wrench[:3]
        moment = wrench[-3:]

        # init output
        joint_torques = [moment[-1]]

        # loop through links from flange to base
        # each iteration calculates for link i-1
        for i, p in reversed(list(enumerate(q))):
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
        return np.ndarray(list(reversed(joint_torques)), dtype=float)


class RobotJSONEncoder(JSONEncoder):
    """Robot JSON Encoder class."""

    def default(self, o: Any) -> Any:
        """
        Return serializable robot objects.

        :param o:
        :return:
        """
        # process custom instances
        if isinstance(o, np.ndarray):
            return o.tolist()

        if isinstance(o, np.int64):
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
    actual_vector = _matrix_2_euler_zyx(actual_pose)

    desired_vector = _matrix_2_euler_zyx(pose
                                         )
    diff = np.abs(actual_vector - desired_vector)
    # diff = np.abs(actual_pose - pose)

    return diff.ravel()


def compute_absolute_errors(qs: np.ndarray,
                            positions: np.ndarray,
                            robot: Robot
                            ) -> np.ndarray:
    """
    Compute the absolute errors of a given set of positions.

    :param robot: robot model
    :param qs: sequence of link positions (e.g., joints)
    :param positions: sequence of actual XYZ positions
    :return:
    """
    # ensure array of arrays
    if qs.ndim == 1:
        qs = np.expand_dims(qs, axis=0)
    if positions.ndim == 1:
        positions = np.expand_dims(positions, axis=0)

    # compute fk positions
    actual_poses = np.array(list(map(robot.fk, qs)))
    actual_positions = actual_poses[:, :-1, -1]

    # compute error
    position_errors = positions - actual_positions
    distance_errors = np.linalg.norm(position_errors, axis=1)

    return distance_errors
