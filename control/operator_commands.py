"""Thread-safe semantic operator input for headless flight sessions."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from time import monotonic
from typing import AbstractSet, Deque, Optional


class OperatorCommand(str, Enum):
    """Commands shared by window, HTTP, and future hardware controllers."""

    SELECT_MANUAL = "select_manual"
    SELECT_NORMAL = "select_normal"
    SELECT_SIDE = "select_side"
    SELECT_FRONT = "select_front"
    TOGGLE_PAUSE = "toggle_pause"
    MOVE_FORWARD = "move_forward"
    MOVE_BACKWARD = "move_backward"
    MOVE_LEFT = "move_left"
    MOVE_RIGHT = "move_right"
    MOVE_UP = "move_up"
    MOVE_DOWN = "move_down"
    YAW_LEFT = "yaw_left"
    YAW_RIGHT = "yaw_right"
    HOVER = "hover"
    STOP = "stop"
    EMERGENCY_STOP = "emergency_stop"


@dataclass(frozen=True)
class OperatorCommandEnvelope:
    """One locally sequenced command received from an operator interface."""

    sequence: int
    command: OperatorCommand
    received_at: float


class OperatorCommandChannel:
    """Bounded in-process mailbox with priority for stop commands."""

    _SAFETY_COMMANDS = {OperatorCommand.STOP, OperatorCommand.EMERGENCY_STOP}

    def __init__(self, capacity: int = 32) -> None:
        if capacity < 1:
            raise ValueError("operator command capacity must be positive")
        self._capacity = int(capacity)
        self._pending: Deque[OperatorCommandEnvelope] = deque()
        self._sequence = 0
        self._lock = Lock()

    def submit(self, command: OperatorCommand) -> OperatorCommandEnvelope:
        """Queue a command; safety commands discard stale pending motion."""

        if not isinstance(command, OperatorCommand):
            raise TypeError("command must be an OperatorCommand")
        with self._lock:
            self._sequence += 1
            envelope = OperatorCommandEnvelope(
                sequence=self._sequence,
                command=command,
                received_at=monotonic(),
            )
            if command in self._SAFETY_COMMANDS:
                self._pending.clear()
            elif len(self._pending) >= self._capacity:
                self._pending.popleft()
            self._pending.append(envelope)
            return envelope

    def receive(
        self,
        allowed_commands: Optional[AbstractSet[OperatorCommand]] = None,
    ) -> Optional[OperatorCommandEnvelope]:
        """Return the oldest allowed command without disturbing deferred input."""

        with self._lock:
            if not self._pending:
                return None
            if allowed_commands is None:
                return self._pending.popleft()
            for index, envelope in enumerate(self._pending):
                if envelope.command in allowed_commands:
                    del self._pending[index]
                    return envelope
            return None

    def clear(self) -> None:
        with self._lock:
            self._pending.clear()

    @property
    def pending_count(self) -> int:
        with self._lock:
            return len(self._pending)
