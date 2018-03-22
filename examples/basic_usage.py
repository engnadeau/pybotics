from pybotics import Tool
from pybotics.geometry import vector_2_matrix
from pybotics.predefined_models import UR10

if __name__ == '__main__':
    # init robot
    robot = UR10()

    # add tool
    tool = Tool()
    tool.position = [1, 2, 3]
    robot.tool = tool

    # set world frame
    world_frame = vector_2_matrix([100, 200, 300, 0, 0, 0])
    robot.world_frame = world_frame

    # print debug info
    print('Robot: {}'.format(robot))
    print('Kinematic Chain: {}'.format(robot.kinematic_chain.to_dict()))
