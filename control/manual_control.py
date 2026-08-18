"""Timed keyboard manual-control state with conservative flight guards."""

from __future__ import annotations

from dataclasses import dataclass, replace
from math import isfinite
from typing import Any, Dict, Optional

from control.follow_control import RCCommand
from drone.safety import SafetyManager


@dataclass(frozen=True)
class ManualControlConfig:
    """Configuration for operator takeover after the base-height climb."""

    enabled: bool = False
    forward_speed: int = 20
    lateral_speed: int = 20
    vertical_speed: int = 15
    yaw_speed: int = 20
    command_timeout_seconds: float = 0.25
    mode_switch_debounce_seconds: float = 0.75
    reacquire_frames: int = 5
    front_tof_guard_enabled: bool = True
    front_stop_distance_cm: float = 60.0
    block_forward_when_tof_invalid: bool = True
    minimum_descent_height_cm: int = 40
    maximum_ascent_height_cm: int = 200

    @classmethod
    def from_dict(cls, data: Dict[str, object]) -> "ManualControlConfig":
        raw = data.get("manual_control", {}) if isinstance(data, dict) else {}
        config = raw if isinstance(raw, dict) else {}
        minimum_descent_height_cm = max(
            1, _as_int(config.get("minimum_descent_height_cm"), 40)
        )
        global_max_height_cm = max(1, _as_int(data.get("max_height_cm"), 220))
        maximum_ascent_height_cm = min(
            global_max_height_cm,
            max(1, _as_int(config.get("maximum_ascent_height_cm"), 200)),
        )
        if maximum_ascent_height_cm <= minimum_descent_height_cm:
            minimum_descent_height_cm = max(1, maximum_ascent_height_cm - 1)
        return cls(
            enabled=_as_bool(config.get("enabled"), False),
            forward_speed=_positive_int(config.get("forward_speed"), 20),
            lateral_speed=_positive_int(config.get("lateral_speed"), 20),
            vertical_speed=_positive_int(config.get("vertical_speed"), 15),
            yaw_speed=_positive_int(config.get("yaw_speed"), 20),
            command_timeout_seconds=max(
                0.05, _as_float(config.get("command_timeout_seconds"), 0.25)
            ),
            mode_switch_debounce_seconds=max(
                0.0, _as_float(config.get("mode_switch_debounce_seconds"), 0.75)
            ),
            reacquire_frames=_positive_int(config.get("reacquire_frames"), 5),
            front_tof_guard_enabled=_as_bool(
                config.get("front_tof_guard_enabled"), True
            ),
            front_stop_distance_cm=max(
                1.0, _as_float(config.get("front_stop_distance_cm"), 60.0)
            ),
            block_forward_when_tof_invalid=_as_bool(
                config.get("block_forward_when_tof_invalid"), True
            ),
            minimum_descent_height_cm=minimum_descent_height_cm,
            maximum_ascent_height_cm=maximum_ascent_height_cm,
        )


class ManualControlController:
    """Convert short-lived key events into guarded RC commands.

    OpenCV exposes key events but no dependable key-up signal.  Each direction
    key therefore creates a short command lease; holding/repeating the key
    refreshes the lease and silence always degrades to hover.
    """

    KEY_COMMANDS = {
        ord("w"): "forward",
        ord("W"): "forward",
        ord("s"): "backward",
        ord("S"): "backward",
        ord("a"): "left",
        ord("A"): "left",
        ord("d"): "right",
        ord("D"): "right",
        ord("r"): "up",
        ord("R"): "up",
        ord("f"): "down",
        ord("F"): "down",
        ord("j"): "yaw_left",
        ord("J"): "yaw_left",
        ord("l"): "yaw_right",
        ord("L"): "yaw_right",
    }

    def __init__(
        self,
        config: ManualControlConfig,
        safety_manager: SafetyManager,
    ) -> None:
        self.config = config
        self.safety_manager = safety_manager
        self.available = False
        self.active = False
        self._command = RCCommand()
        self._expires_at = 0.0
        self.last_guard_reason = ""

    @classmethod
    def from_config(
        cls,
        data: Dict[str, object],
        safety_manager: SafetyManager,
    ) -> "ManualControlController":
        config = ManualControlConfig.from_dict(data)
        safety_ceiling = max(1, int(safety_manager.config.max_height_cm))
        maximum_ascent = min(config.maximum_ascent_height_cm, safety_ceiling)
        minimum_descent = config.minimum_descent_height_cm
        if maximum_ascent <= minimum_descent:
            minimum_descent = max(1, maximum_ascent - 1)
        if (
            maximum_ascent != config.maximum_ascent_height_cm
            or minimum_descent != config.minimum_descent_height_cm
        ):
            config = replace(
                config,
                minimum_descent_height_cm=minimum_descent,
                maximum_ascent_height_cm=maximum_ascent,
            )
        return cls(config, safety_manager)

    def make_available(self) -> None:
        """Allow takeover only after the base-height phase completes."""
        self.available = bool(self.config.enabled)

    def enable(self, now: float) -> bool:
        if not self.available or not self.config.enabled:
            return False
        self.active = True
        self._command = RCCommand()
        self._expires_at = float(now)
        self.last_guard_reason = "waiting for operator input"
        return True

    def disable(self) -> None:
        self.active = False
        self._command = RCCommand()
        self._expires_at = 0.0
        self.last_guard_reason = ""

    def force_hover(self, reason: str = "operator hover") -> None:
        """Invalidate the active direction lease without leaving manual mode."""
        self._command = RCCommand()
        self._expires_at = 0.0
        self.last_guard_reason = str(reason)

    def reset(self) -> None:
        self.available = False
        self.disable()

    def handle_key(self, key: int, now: float) -> bool:
        """Consume one manual motion key; return whether it was recognized."""
        if not self.active:
            return False
        if key == ord(" "):
            self._command = RCCommand()
            self._expires_at = float(now)
            self.last_guard_reason = "operator hover"
            return True
        action = self.KEY_COMMANDS.get(key)
        if action is None:
            return False
        self._command = self._command_for_action(action)
        self._expires_at = float(now) + self.config.command_timeout_seconds
        self.last_guard_reason = ""
        return True

    def command_for(
        self,
        *,
        now: float,
        height_cm: Optional[float],
        front_tof_snapshot: Optional[Any],
    ) -> RCCommand:
        """Return the current command after timeout, ToF, and height guards."""
        if not self.active:
            return RCCommand()
        if float(now) >= self._expires_at:
            self._command = RCCommand()
            if not self.last_guard_reason:
                self.last_guard_reason = "manual command timed out"
            return RCCommand()

        command = self._command
        left_right = command.left_right
        forward_backward = command.forward_backward
        up_down = command.up_down
        yaw = command.yaw
        guard_reasons = []

        if forward_backward > 0 and self.config.front_tof_guard_enabled:
            blocked_reason = self._front_guard_reason(front_tof_snapshot)
            if blocked_reason:
                forward_backward = 0
                guard_reasons.append(blocked_reason)

        if height_cm is not None:
            if (
                up_down > 0
                and float(height_cm) >= self.config.maximum_ascent_height_cm
            ):
                up_down = 0
                guard_reasons.append("manual ascent height limit")
            elif (
                up_down < 0
                and float(height_cm) <= self.config.minimum_descent_height_cm
            ):
                up_down = 0
                guard_reasons.append("manual descent height limit")

        limited = self.safety_manager.limit_rc_command(
            left_right, forward_backward, up_down, yaw
        )
        self.last_guard_reason = "; ".join(guard_reasons)
        return RCCommand(*limited)

    def _front_guard_reason(self, snapshot: Optional[Any]) -> str:
        if snapshot is None:
            return (
                "front ToF unavailable"
                if self.config.block_forward_when_tof_invalid
                else ""
            )
        status = str(getattr(snapshot, "status", "not_ready")).strip().lower()
        if status == "out_of_range":
            return ""
        if status != "valid":
            return (
                f"front ToF {status or 'invalid'}"
                if self.config.block_forward_when_tof_invalid
                else ""
            )
        distance = getattr(snapshot, "distance_cm", None)
        if distance is None:
            return (
                "front ToF missing distance"
                if self.config.block_forward_when_tof_invalid
                else ""
            )
        try:
            distance_value = float(distance)
        except (TypeError, ValueError):
            distance_value = float("nan")
        if not isfinite(distance_value) or distance_value <= 0:
            return (
                "front ToF invalid distance"
                if self.config.block_forward_when_tof_invalid
                else ""
            )
        if distance_value <= self.config.front_stop_distance_cm:
            return f"front ToF blocked at {distance_value:.0f} cm"
        return ""

    def _command_for_action(self, action: str) -> RCCommand:
        if action == "forward":
            return RCCommand(forward_backward=self.config.forward_speed)
        if action == "backward":
            return RCCommand(forward_backward=-self.config.forward_speed)
        if action == "left":
            return RCCommand(left_right=-self.config.lateral_speed)
        if action == "right":
            return RCCommand(left_right=self.config.lateral_speed)
        if action == "up":
            return RCCommand(up_down=self.config.vertical_speed)
        if action == "down":
            return RCCommand(up_down=-self.config.vertical_speed)
        if action == "yaw_left":
            return RCCommand(yaw=-self.config.yaw_speed)
        if action == "yaw_right":
            return RCCommand(yaw=self.config.yaw_speed)
        return RCCommand()


def _as_bool(value: object, default: bool) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return default


def _as_int(value: object, default: int) -> int:
    try:
        return int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default


def _positive_int(value: object, default: int) -> int:
    return max(1, _as_int(value, default))


def _as_float(value: object, default: float) -> float:
    try:
        return float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
