from itertools import chain
from typing import Iterable, Optional, Collection

import numpy as np
from pybotics.kinematics.convention import Convention
from pybotics.kinematics.link import Link

from pybotics.revolute_mdh_link import RevoluteMDHLink
from pybotics.vector import Vector


class KinematicChain(Vector):
    @property
    def vector(self) -> np.ndarray:
        return np.array(list(chain(self.links)))

    def __init__(self, links: Iterable[Link] = None, array: Optional[np.ndarray] = None,
                 convention: Convention = None) -> None:
        # init private variables
        self._links = None

        if links is None:
            if array is None or convention is None:
                raise ValueError('array and convention must be set if links is None')
            else:
                if convention is Convention.MDH:
                    self.links = [RevoluteMDHLink(*row) for row in array]
                elif convention in Convention:
                    raise NotImplementedError('{} has not been implemented'.format(convention))
                else:
                    raise ValueError('Supported conventions: {}'.format([e for e in Convention]))
        else:
            self.links = links

    @property
    def links(self) -> Collection[Link]:
        return self._links

    @links.setter
    def links(self, value: Iterable[Link]):
        KinematicChain.validate_link_conventions(value)
        self._links = value

    def transforms(self, positions=None) -> Iterable[np.ndarray]:
        positions = np.zeros(self.num_dof()) if positions is None else positions
        return [link.transform(p) for link, p in zip(self.links, positions)]

    def num_dof(self) -> int:
        return len(self.links)

    @staticmethod
    def validate_link_conventions(links: Iterable[Link]):
        convention = None
        for link in links:
            if convention is None:
                convention = link.convention
            else:
                if link.convention is not convention:
                    raise ValueError('All links must use the same convention')

    def convention(self):
        return self.links[0].convention
