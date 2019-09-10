"""Pybotics errors."""
import attr


@attr.s
class PyboticsError(Exception):
    """Base class for Pybotics errors."""

    message = attr.ib("Pybotics error", type=str)

    def __str__(self) -> str:
        """
        Convert exception to string.

        :return: string
        """
        return self.message
