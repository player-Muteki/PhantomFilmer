"""Standard high-level console actions."""

from enum import Enum


class ConsoleCommand(str, Enum):
    """Whitelisted high-level actions the console may execute."""

    GET_STATUS = "GET_STATUS"
    START_FOLLOW = "START_FOLLOW"
    STOP_TASK = "STOP_TASK"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    EXIT = "EXIT"
    UNKNOWN = "UNKNOWN"
