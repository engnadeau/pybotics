import numpy as np
from pybotics.kinematics.convention import Convention
from pybotics.kinematics.revolute_mdh_link import RevoluteMDHLink


class KinematicChain:
    def __init__(self, links) -> None:
        self.links = links

    def transforms(self, positions=None):
        positions = np.zeros(self.num_dof()) if positions is None else positions
        return [link.transform(p) for link, p in zip(self.links, positions)]

    @staticmethod
    def from_array(array, convention):
        if convention is Convention.REVOLUTE_MDH:
            return KinematicChain.from_revolute_mdh(array)
        else:
            raise ValueError('Supported conventions: {}'.format([e for e in Convention]))

    @staticmethod
    def from_revolute_mdh(array):
        links = [RevoluteMDHLink(*row) for row in array]
        return KinematicChain(links)

    def num_dof(self):
        return len(self.links)
