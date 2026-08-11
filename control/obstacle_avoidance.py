"""Obstacle-avoidance command arbitration for visual follow."""

from dataclasses import dataclass
from time import monotonic
from typing import Dict, Optional

from control.follow_control import RCCommand
from drone.safety import SafetyManager
from vision.obstacle_detect import ObstacleResult


@dataclass
class AvoidanceDecision:
    """Final command and state selected by obstacle avoidance."""

    command: RCCommand
    state: str = "CLEAR"
    reason: str = ""


class ObstacleAvoidancePlanner:
    """Conservatively override forward follow commands when obstacle risk is high."""

    def __init__(
        self,
        safety_manager: SafetyManager,
        avoidance_yaw_speed: int = 18,
        avoidance_lateral_speed: int = 0,
        max_avoidance_seconds: float = 5.0,
        recovery_clear_frames: int = 10,
        forward_speed_in_caution_ratio: float = 0.35,
    ) -> None:
        self.safety_manager = safety_manager
        self.avoidance_yaw_speed = self._non_negative_int(avoidance_yaw_speed, 18)
        self.avoidance_lateral_speed = self._non_negative_int(avoidance_lateral_speed, 0)
        self.max_avoidance_seconds = self._positive_float(max_avoidance_seconds, 5.0)
        self.recovery_clear_frames = self._positive_int(recovery_clear_frames, 10)
        self.forward_speed_in_caution_ratio = self._clamp_float(
            forward_speed_in_caution_ratio, 0.0, 1.0, 0.35
        )
        self.state = "CLEAR"
        self._clear_frames = 0
        self._avoidance_started_at: Optional[float] = None
        self._last_direction = "right"

    @classmethod
    def from_config(
        cls,
        safety_manager: SafetyManager,
        config: Dict[str, object],
    ) -> "ObstacleAvoidancePlanner":
        """Build a planner from config.yaml."""
        obstacle = config.get("obstacle", {}) if isinstance(config, dict) else {}
        if not isinstance(obstacle, dict):
            obstacle = {}
        return cls(
            safety_manager=safety_manager,
            avoidance_yaw_speed=cls._config_int(obstacle, "avoidance_yaw_speed", 18),
            avoidance_lateral_speed=cls._config_int(obstacle, "avoidance_lateral_speed", 0),
            max_avoidance_seconds=cls._config_float(obstacle, "max_avoidance_seconds", 5.0),
            recovery_clear_frames=cls._config_int(obstacle, "recovery_clear_frames", 10),
            forward_speed_in_caution_ratio=cls._config_float(
                obstacle, "forward_speed_in_caution_ratio", 0.35
            ),
        )

    def plan(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
    ) -> AvoidanceDecision:
        """Return a final command after applying obstacle priority rules."""
        if not obstacle_result.found:
            return self._plan_clear(follow_command)

        self._clear_frames = 0
        if obstacle_result.side in ("left", "right"):
            self._last_direction = "right" if obstacle_result.side == "left" else "left"

        if obstacle_result.state == "CAUTION":
            self.state = "CAUTION"
            command = self._caution_command(follow_command)
            return AvoidanceDecision(command=command, state=self.state, reason="obstacle caution")

        return self._plan_blocked(follow_command, obstacle_result)

    def reset(self) -> None:
        """Clear avoidance state for a new follow session."""
        self.state = "CLEAR"
        self._clear_frames = 0
        self._avoidance_started_at = None
        self._last_direction = "right"

    def _plan_clear(self, follow_command: RCCommand) -> AvoidanceDecision:
        if self.state in ("BLOCKED", "AVOIDING", "RECOVERING", "CAUTION"):
            self._clear_frames += 1
            if self._clear_frames < self.recovery_clear_frames:
                self.state = "RECOVERING"
                command = self._limited_command(0, 0, follow_command.up_down, follow_command.yaw)
                return AvoidanceDecision(command=command, state=self.state, reason="recovering after obstacle")

        self.state = "CLEAR"
        self._clear_frames = 0
        self._avoidance_started_at = None
        return AvoidanceDecision(command=self._limit(follow_command), state=self.state, reason="clear")

    def _plan_blocked(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
    ) -> AvoidanceDecision:
        now = monotonic()
        if self._avoidance_started_at is None:
            self._avoidance_started_at = now
        elapsed = now - self._avoidance_started_at
        if elapsed > self.max_avoidance_seconds:
            self.state = "BLOCKED"
            command = self._limited_command(0, 0, follow_command.up_down, 0)
            return AvoidanceDecision(command=command, state=self.state, reason="avoidance timeout")

        self.state = "AVOIDING"
        direction = self._avoidance_direction(obstacle_result.side)
        lateral = self.avoidance_lateral_speed if direction == "right" else -self.avoidance_lateral_speed
        yaw = self.avoidance_yaw_speed if direction == "right" else -self.avoidance_yaw_speed
        command = self._limited_command(lateral, 0, follow_command.up_down, yaw)
        return AvoidanceDecision(command=command, state=self.state, reason=f"avoiding {direction}")

    def _caution_command(self, follow_command: RCCommand) -> RCCommand:
        forward = follow_command.forward_backward
        if forward > 0:
            forward = int(forward * self.forward_speed_in_caution_ratio)
        return self._limited_command(
            follow_command.left_right,
            forward,
            follow_command.up_down,
            follow_command.yaw,
        )

    def _avoidance_direction(self, side: str) -> str:
        if side == "left":
            self._last_direction = "right"
        elif side == "right":
            self._last_direction = "left"
        return self._last_direction

    def _limit(self, command: RCCommand) -> RCCommand:
        return self._limited_command(*command.as_tuple())

    def _limited_command(
        self,
        left_right: int,
        forward_backward: int,
        up_down: int,
        yaw: int,
    ) -> RCCommand:
        limited = self.safety_manager.limit_rc_command(left_right, forward_backward, up_down, yaw)
        return RCCommand(*limited)

    @staticmethod
    def _clamp_float(value: float, lower: float, upper: float, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        return max(lower, min(upper, numeric))

    @staticmethod
    def _positive_float(value: float, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        return max(0.0001, numeric)

    @staticmethod
    def _positive_int(value: int, default: int) -> int:
        try:
            numeric = int(value)
        except (TypeError, ValueError):
            numeric = default
        return max(1, numeric)

    @staticmethod
    def _non_negative_int(value: int, default: int) -> int:
        try:
            numeric = int(value)
        except (TypeError, ValueError):
            numeric = default
        return max(0, numeric)

    @staticmethod
    def _config_float(config: Dict[str, object], key: str, default: float) -> float:
        try:
            return float(config.get(key, default))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _config_int(config: Dict[str, object], key: str, default: int) -> int:
        try:
            return int(config.get(key, default))
        except (TypeError, ValueError):
            return default
