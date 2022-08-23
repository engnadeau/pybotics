"""Basic usage of the pybotics package."""
from pybotics.geometry import vector_2_matrix
from pybotics.predefined_models import ur10
from pybotics.robot import Robot
from pybotics.tool import Tool
import numpy as np


def main() -> None:
    """
    Demonstrate pybotics usage.

    View source for more info.
    """
    # init robot
    robot = Robot.from_parameters(ur10())

    # add tool
    tool = Tool()
    tool.position = np.array([1, 2, 3])
    robot.tool = tool

    # set world frame
    world_frame = vector_2_matrix(np.array([100, 200, 300, 0, 0, 0]))
    robot.world_frame = world_frame

    print(f"Robot: {robot}")
    print(f"Kinematic Chain: {robot.kinematic_chain}")


if __name__ == "__main__":
    main()
