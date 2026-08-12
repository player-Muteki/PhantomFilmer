"""Unified abstract drone adapter interface used by the control system."""

from abc import ABC, abstractmethod
from typing import Any


class DroneAdapter(ABC):
    """Base interface for real or simulated drone backends.

    Other modules should depend on this interface instead of importing a
    hardware SDK directly.
    """

    @abstractmethod
    def connect(self) -> None:
        """Connect to the drone."""

    @abstractmethod
    def takeoff(self) -> None:
        """Command the drone to take off after explicit user confirmation."""

    @abstractmethod
    def land(self) -> None:
        """Command the drone to land."""

    @abstractmethod
    def stop(self) -> None:
        """Stop motion and release drone resources."""

    @abstractmethod
    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        """Send remote-control velocity commands to the drone."""

    @abstractmethod
    def get_battery(self) -> int:
        """Return the current battery percentage."""

    @abstractmethod
    def get_height(self) -> int:
        """Return downward ground clearance in centimeters."""

    @abstractmethod
    def stream_on(self) -> None:
        """Enable the drone video stream."""

    @abstractmethod
    def stream_off(self) -> None:
        """Disable the drone video stream."""

    @abstractmethod
    def get_frame(self) -> Any:
        """Return the latest video frame from the drone camera."""
