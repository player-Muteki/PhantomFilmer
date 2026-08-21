"""Typed operator commands accepted by the shared mission runtime."""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from time import time
from typing import Any, Union
from uuid import uuid4

from app.runtime.models import ControlMode, MissionKind


@dataclass(frozen=True, kw_only=True)
class CommandMetadata:
    """Identity and creation time shared by every runtime command."""

    command_id: str = field(default_factory=lambda: uuid4().hex)
    issued_at: float = field(default_factory=time)

    def __post_init__(self) -> None:
        if not self.command_id or len(self.command_id) > 128:
            raise ValueError("commandId 必须是 1～128 个字符。")
        if not isfinite(float(self.issued_at)):
            raise ValueError("issuedAt 时间戳无效。")


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
    profile_name: str | None = None
    initial_control_mode: ControlMode = ControlMode.NORMAL
    obstacle_enabled: bool | None = None


@dataclass(frozen=True)
class StopMissionCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class EmergencyStopCommand(CommandMetadata):
    pass


@dataclass(frozen=True)
class SelectControlModeCommand(CommandMetadata):
    mode: ControlMode


@dataclass(frozen=True)
class ToggleMissionPauseCommand(CommandMetadata):
    pass


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
    ToggleMissionPauseCommand,
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
        ToggleMissionPauseCommand: "mission.pause.toggle",
    }
    return names[type(command)]


def command_from_payload(payload: dict[str, Any]) -> RuntimeCommand:
    """Parse the versioned JSON command envelope into one typed command."""

    command_type = payload.get("type")
    if not isinstance(command_type, str):
        raise ValueError("命令 type 缺失或格式无效。")
    metadata: dict[str, Any] = {}
    command_id = payload.get("commandId")
    if command_id is not None:
        if not isinstance(command_id, str):
            raise ValueError("commandId 格式无效。")
        metadata["command_id"] = command_id
    issued_at = payload.get("issuedAt")
    if issued_at is not None:
        if isinstance(issued_at, bool) or not isinstance(issued_at, (int, float)):
            raise ValueError("issuedAt 格式无效。")
        metadata["issued_at"] = float(issued_at) / 1000.0

    constructors = {
        "device.connect": ConnectCommand,
        "device.status.refresh": RefreshStatusCommand,
        "flight.takeoff": TakeoffCommand,
        "flight.land": LandCommand,
        "flight.hover": HoverCommand,
        "device.stop": StopCommand,
        "flight.emergency_land": EmergencyLandCommand,
        "mission.stop": StopMissionCommand,
        "mission.emergency_stop": EmergencyStopCommand,
        "mission.pause.toggle": ToggleMissionPauseCommand,
    }
    constructor = constructors.get(command_type)
    if constructor is not None:
        return constructor(**metadata)
    if command_type == "mission.start":
        try:
            mission = MissionKind(str(payload.get("mission")))
        except ValueError as exc:
            raise ValueError("mission 类型无效。") from exc
        profile_name = payload.get("profileName")
        if profile_name is not None:
            if not isinstance(profile_name, str) or not profile_name.strip():
                raise ValueError("profileName 格式无效。")
            profile_name = profile_name.strip()
        try:
            initial_control_mode = ControlMode(
                str(payload.get("initialControlMode", ControlMode.NORMAL.value))
            )
        except ValueError as exc:
            raise ValueError("initialControlMode 格式无效。") from exc
        if initial_control_mode is ControlMode.NONE:
            raise ValueError("自动任务的 initialControlMode 不能为 none。")
        obstacle_enabled = payload.get("obstacleEnabled")
        if obstacle_enabled is not None and not isinstance(obstacle_enabled, bool):
            raise ValueError("obstacleEnabled 格式无效。")
        return StartMissionCommand(
            mission=mission,
            profile_name=profile_name,
            initial_control_mode=initial_control_mode,
            obstacle_enabled=obstacle_enabled,
            **metadata,
        )
    if command_type == "mission.control_mode.select":
        try:
            mode = ControlMode(str(payload.get("mode")))
        except ValueError as exc:
            raise ValueError("控制模式无效。") from exc
        return SelectControlModeCommand(mode=mode, **metadata)
    if command_type == "flight.rc.move":
        raise ValueError("flight.rc.move 必须通过带租约的 /api/v1/rc 接口发送。")
    raise ValueError(f"未知命令类型：{command_type}")
