import numpy as np


def compute_absolute_errors(robot, joints, torques, positions, reference_frame):
    assert len(joints) == len(torques)
    assert len(joints) == len(positions)

    errors = []
    for i, _ in enumerate(joints):
        pose = robot.fk(joints[i], torques=torques[i], reference_frame=reference_frame)
        pose_xyz = pose[0:-1, -1]

        position_error = pose_xyz - positions[i]
        error = np.linalg.norm(position_error)
        errors.append(error)

    errors = np.abs(errors)
    return errors


def calibration_fitness_func(optimization_vector,
                             optimization_mask,
                             robot,
                             joints,
                             torques,
                             positions,
                             reference_frame,
                             calibration_type,
                             return_type
                             ):
    # validate input
    assert len(joints) == len(torques)
    assert len(joints) == len(positions)

    # make sure optimization_vector is a list
    if isinstance(optimization_vector, np.ndarray):
        optimization_vector = optimization_vector.tolist()

    robot.apply_optimization_vector(optimization_vector, optimization_mask)

    if calibration_type == 'abs':
        errors = compute_absolute_errors(robot, joints, torques, positions, reference_frame)
    elif calibration_type == 'rel':
        # TODO: implement relative errors
        # errors = compute_relative_errors(robot, joints, torques, positions)
        pass
    else:
        raise ValueError

    if return_type == 'list':
        return errors
    elif return_type == 'sumsq':
        return np.sum(np.square(errors))
    else:
        raise ValueError
