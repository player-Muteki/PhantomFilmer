"""Stable internal state model shared by operator interfaces."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Mapping, Optional, Tuple


class RuntimePhase(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    PREFLIGHT = "preflight"
    TAKING_OFF = "taking_off"
    AIRBORNE = "airborne"
    LANDING = "landing"
    STOPPING = "stopping"
    ERROR = "error"


class MissionKind(str, Enum):
    IDLE = "idle"
    MANUAL = "manual"
    FOLLOW = "follow"
    REID_FOLLOW = "reid_follow"
    DRY_RUN = "dry_run"


class ControlMode(str, Enum):
    NONE = "none"
    MANUAL = "manual"
    NORMAL = "normal"
    SIDE = "side"
    FRONT = "front"


class AllowedAction(str, Enum):
    CONNECT = "connect"
    REFRESH_STATUS = "refresh_status"
    TAKEOFF = "takeoff"
    LAND = "land"
    HOVER = "hover"
    MOVE_RC = "move_rc"
    STOP = "stop"
    EMERGENCY_LAND = "emergency_land"
    START_MISSION = "start_mission"
    STOP_MISSION = "stop_mission"
    EMERGENCY_STOP_MISSION = "emergency_stop_mission"
    SELECT_CONTROL_MODE = "select_control_mode"
    TOGGLE_MISSION_PAUSE = "toggle_mission_pause"
    START_PREVIEW = "start_preview"
    STOP_PREVIEW = "stop_preview"


@dataclass(frozen=True)
class RuntimeSnapshot:
    """One immutable, serializable view of runtime truth."""

    sequence: int
    phase: RuntimePhase
    mission: MissionKind
    control_mode: ControlMode
    connected: bool
    airborne: bool
    streaming: bool
    flight_state: str
    allowed_actions: Tuple[AllowedAction, ...] = ()
    telemetry: Mapping[str, Any] = field(default_factory=dict)
    error: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        return {
            "sequence": self.sequence,
            "phase": self.phase.value,
            "mission": self.mission.value,
            "controlMode": self.control_mode.value,
            "connected": self.connected,
            "airborne": self.airborne,
            "streaming": self.streaming,
            "flightState": self.flight_state,
            "allowedActions": [action.value for action in self.allowed_actions],
            "telemetry": dict(self.telemetry),
            "error": self.error,
        }
