"""Standard high-level Agent actions."""

from enum import Enum


class AgentCommand(str, Enum):
    """Whitelisted high-level actions the Agent may execute."""

    GET_STATUS = "GET_STATUS"
    START_FOLLOW = "START_FOLLOW"
    STOP_TASK = "STOP_TASK"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    EXIT = "EXIT"
    UNKNOWN = "UNKNOWN"


ALLOWED_AGENT_COMMANDS = {command.value for command in AgentCommand}
