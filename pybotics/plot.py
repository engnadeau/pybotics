from pybotics.predefined_models import UR10
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class PLTRobot(UR10):
    def link_endpoints(self):
        q = self.joints
        xyzs = np.array(
            [tr[:-1, -1] for tr in self.yield_transforms(q)]
        )

        return xyzs

    def yield_transforms(self, q):
        pose = np.eye(4)
        for tr in self.transforms(q):
            pose = np.dot(pose, tr)
            yield pose


if __name__ == '__main__':
    # TODO: when both MDH.a and MDH.d are defined, plot lines are oblique
    # TODO: take MDH parameters into account and split points into a and d components
    np.set_printoptions(suppress=True)
    robot = PLTRobot()
    robot.joints = np.deg2rad([0, 0, 90, 0, 0, 0])
    print(robot.link_endpoints())

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    ax.plot(
        xs=robot.link_endpoints()[:, 0],
        ys=robot.link_endpoints()[:, 1],
        zs=robot.link_endpoints()[:, 2],
        marker='o'
    )
    plt.axis('equal')
    plt.show()
