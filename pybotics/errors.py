"""Pybotics errors."""
from typing import Optional


class PyboticsError(Exception):
    """Base class for Pybotics errors."""

    _default_message = 'Pybotics error'

    def __init__(self, message: Optional[str] = None) -> None:
        """
        Construct base exception.

        :param message:
        """
        self.message = message

    def __str__(self) -> str:
        """
        Convert exception to string.

        :return: string
        """
        if self.message is None:
            return self._default_message
        else:
            return self.message
