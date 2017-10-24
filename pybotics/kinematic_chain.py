from itertools import chain
from typing import List, Iterable, Any, Sequence

import numpy as np  # type: ignore

from pybotics.convention import Convention
from pybotics.link import Link
from pybotics.optimizable import Optimizable
from pybotics.revolute_mdh_link import RevoluteMDHLink
from pybotics.validation import is_same_link_conventions
from pybotics.vector import Vector


class KinematicChain(Vector, Optimizable):
    @property
    def vector(self) -> np.ndarray:
        return np.array(list(chain(self.links)))

    def __init__(self, links: Sequence[Link]) -> None:
        # init private variables
        self._links = links

    @classmethod
    def from_array(cls, array: np.ndarray,
                   convention: Convention) -> Any:
        if convention is Convention.MDH:
            links = [RevoluteMDHLink(*row) for row in array]
        elif convention in Convention:
            raise NotImplementedError(
                '{} has not been implemented'.format(convention))
        else:
            raise ValueError('Supported conventions: {}'.format(
                [e for e in Convention]))

        return cls(links=links)

    @property
    def links(self) -> Sequence[Link]:
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
