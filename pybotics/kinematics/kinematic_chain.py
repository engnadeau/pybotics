from itertools import chain

import numpy as np
from pybotics.kinematics.convention import Convention
from pybotics.kinematics.revolute_mdh_link import RevoluteMDHLink
from pybotics.models.vector import Vector


class KinematicChain(Vector):
    @property
    def vector(self):
        return np.array(list(chain(self.links)))

    def __init__(self, links) -> None:
        self._links = None
        self.links = links

    @property
    def links(self):
        return self._links

    @links.setter
    def links(self, value):
        KinematicChain.validate_link_conventions(value)
        self._links = value

    def transforms(self, positions=None):
        positions = np.zeros(self.num_dof()) if positions is None else positions
        return [link.transform(p) for link, p in zip(self.links, positions)]

    @staticmethod
    def from_array(array, convention):
        if convention is Convention.MDH:
            return KinematicChain.from_revolute_mdh(array)
        elif convention in Convention:
            raise NotImplementedError('{} has not been implemented'.format(convention))
        else:
            raise ValueError('Supported conventions: {}'.format([e for e in Convention]))

    @staticmethod
    def from_revolute_mdh(array):
        links = [RevoluteMDHLink(*row) for row in array]
        return KinematicChain(links)

    def num_dof(self):
        return len(self.links)

    @staticmethod
    def validate_link_conventions(links):
        convention = None
        for link in links:
            if convention is None:
                convention = link.convention
            else:
                if link.convention is not convention:
                    raise ValueError('All links must use the same convention')

    def convention(self):
        return self.links[0].convention
