import numpy as np


def random_joints(joint_angle_limits):
    joint_angles = []
    for limits in joint_angle_limits:
        joint_angles.append(np.random.uniform(min(limits), max(limits)))
    return np.array(joint_angles)
