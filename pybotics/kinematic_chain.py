from itertools import chain
from typing import Optional, List, Iterable, Union

import numpy as np
from pybotics.convention import Convention
from pybotics.link import Link

from pybotics.revolute_mdh_link import RevoluteMDHLink
from pybotics.validation import is_same_link_conventions
from pybotics.vector import Vector


class KinematicChain(Vector):
    @property
    def vector(self) -> np.ndarray:
        return np.array(list(chain(self.links)))

    def __init__(self, links: Union[List[Link], np.ndarray] = None, convention: Convention = None) -> None:
        # init private variables
        self._links = None

        if isinstance(links, np.ndarray):
            if convention is None:
                raise ValueError('convention must be set if isinstance(links, np.ndarray)')
            else:
                if convention is Convention.MDH:
                    self.links = [RevoluteMDHLink(*row) for row in links]
                elif convention in Convention:
                    raise NotImplementedError('{} has not been implemented'.format(convention))
                else:
                    raise ValueError('Supported conventions: {}'.format([e for e in Convention]))
        else:
            self.links = links

    @property
    def links(self) -> List[Link]:
        return self._links

    @links.setter
    def links(self, value: List[Link]) -> None:
        if not is_same_link_conventions(value):
            raise ValueError('All links must use the same convention')
        self._links = value

    def transforms(self, positions: Iterable[float]) -> List[np.ndarray]:
        return [link.transform(p) for link, p in zip(self.links, positions)]

    def num_dof(self) -> int:
        return len(self.links)

    def convention(self) -> Convention:
        return self.links[0].convention
