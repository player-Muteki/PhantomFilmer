"""Typed operator commands accepted by the shared mission runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from time import time
from typing import Union
from uuid import uuid4

from app.runtime.models import ControlMode, MissionKind


@dataclass(frozen=True, kw_only=True)
class CommandMetadata:
    """Identity and creation time shared by every runtime command."""

    command_id: str = field(default_factory=lambda: uuid4().hex)
    issued_at: float = field(default_factory=time)


@dataclass(frozen=True)
class ConnectCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class RefreshStatusCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class TakeoffCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class LandCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class HoverCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class MoveRcCommand(CommandMetadata):
    left_right: int = 0
    forward_back: int = 0
    up_down: int = 0
    yaw: int = 0


@dataclass(frozen=True)
class StopCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class EmergencyLandCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class StartMissionCommand(CommandMetadata):
    mission: MissionKind


@dataclass(frozen=True)
class StopMissionCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class EmergencyStopCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class SelectControlModeCommand(CommandMetadata):
    mode: ControlMode


RuntimeCommand = Union[
    ConnectCommand,
    RefreshStatusCommand,
    TakeoffCommand,
    LandCommand,
    HoverCommand,
    MoveRcCommand,
    StopCommand,
    EmergencyLandCommand,
    StartMissionCommand,
    StopMissionCommand,
    EmergencyStopCommand,
    SelectControlModeCommand,
]


def command_name(command: RuntimeCommand) -> str:
    """Return a stable wire/log name without coupling to Python class names."""

    names = {
        ConnectCommand: "device.connect",
        RefreshStatusCommand: "device.status.refresh",
        TakeoffCommand: "flight.takeoff",
        LandCommand: "flight.land",
        HoverCommand: "flight.hover",
        MoveRcCommand: "flight.rc.move",
        StopCommand: "device.stop",
        EmergencyLandCommand: "flight.emergency_land",
        StartMissionCommand: "mission.start",
        StopMissionCommand: "mission.stop",
        EmergencyStopCommand: "mission.emergency_stop",
        SelectControlModeCommand: "mission.control_mode.select",
    }
    return names[type(command)]
