import logging

import numpy as np
from pybotics.robot_model import UR10

if __name__ == '__main__':
    # logging.basicConfig(level=logging.INFO)
    logging.basicConfig(level=logging.DEBUG)
    np.set_printoptions(suppress=True)

    robot = UR10()
    q = np.ones(robot.num_dof)
    q_seed = 0.9 * q
    logging.info('Desired joints: {}'.format(q))

    logging.info('Seed joints: {}'.format(q_seed))

    pose = robot.fk(q)
    logging.info('Desired pose:\n{}'.format(pose))

    robot.ik_jacobian(pose, q=q_seed, alpha=0.8, max_iter=5)
