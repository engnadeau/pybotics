"""Exceptions used in pybotics."""
from typing import Any


class PybotException(Exception):
    """Pybotics exception base class."""

    def __init__(self, *args: Any) -> None:
        """Contruct exception."""
        super().__init__(*args)
