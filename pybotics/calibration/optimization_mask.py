from typing import List, NamedTuple, Union


class OptimizationMask(NamedTuple):
    world_frame: Union[List[bool], bool] = False
    robot_model: Union[List[bool], bool] = False
    tool: Union[List[bool], bool] = False
    joint_compliances: Union[List[bool], bool] = False
