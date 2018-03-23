"""JSON Encoder for Pybotics classes."""
import json
from typing import Any

import numpy as np  # type: ignore


class JSONEncoder(json.JSONEncoder):
    """Pybotics JSON Encoder class."""

    def default(self, o: Any) -> Any:  # pragma: no cover
        """Return serializable robot objects."""
        # TODO: use @overload to split function
        # BODY: Reduces cyclomatic complexity; but requires NumPy typing
        if isinstance(o, np.ndarray):
            return o.tolist()
        elif isinstance(o, np.random.RandomState):
            return None
        elif isinstance(o, np.generic):
            return str(o)
        else:
            try:
                o = o.__dict__
            except AttributeError:
                pass
            else:
                return o

        # let the base class default method raise the TypeError
        # https://docs.python.org/3/library/json.html
        return json.JSONEncoder.default(self, o)
