"""Robot JSON Encoder."""
from json.encoder import JSONEncoder
import numpy as np  # type: ignore
from typing import Any


class RobotJSONEncoder(JSONEncoder):
    """Robot JSON Encoder class."""

    def default(self, o: Any) -> Any:
        """
        Return serializable robot objects.

        :param o:
        :return:
        """
        # process ndarray
        if isinstance(o, np.ndarray):
            o = o.tolist()
            return o

        # process dict-like objects
        try:
            o = o.__dict__
        except AttributeError:
            pass
        else:
            return o

        # let the base class default method raise the TypeError
        # https://docs.python.org/3/library/json.html
        return JSONEncoder.default(self, o)
