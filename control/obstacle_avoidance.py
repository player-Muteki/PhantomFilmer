"""Deterministic three-stage bypass planning for front-ToF observations."""

from dataclasses import dataclass, field
from time import monotonic
from typing import Dict, Optional

from control.follow_control import RCCommand
from drone.safety import SafetyManager
from vision.obstacle_detect import ObstacleResult


@dataclass
class AvoidanceDecision:
    """Final command and explainable state selected by local planning."""

    command: RCCommand
    state: str = "CLEAR"
    reason: str = ""
    action: str = "FOLLOW"
    confidence: float = 1.0
    plan_id: str = "0"
    requires_landing: bool = False
    owns_motion: bool = False
    observation: Optional[ObstacleResult] = field(default=None, repr=False)

    def to_dict(self) -> Dict[str, object]:
        """Return a JSON-serializable decision payload."""
        return {
            "state": self.state,
            "action": self.action,
            "reason": self.reason,
            "confidence": round(self.confidence, 4),
            "plan_id": self.plan_id,
            "requires_landing": self.requires_landing,
            "owns_motion": self.owns_motion,
            "command": {
                "left_right": self.command.left_right,
                "forward_backward": self.command.forward_backward,
                "up_down": self.command.up_down,
                "yaw": self.command.yaw,
            },
        }


class ObstacleAvoidancePlanner:
    """Choose short-horizon local actions without network or model calls."""

    def __init__(
        self,
        safety_manager: SafetyManager,
        avoidance_yaw_speed: int = 18,
        avoidance_lateral_speed: int = 0,
        max_avoidance_seconds: float = 5.0,
        recovery_clear_frames: int = 10,
        detect_confirm_frames: int = 1,
        clear_confirm_frames: Optional[int] = None,
        forward_speed_in_caution_ratio: float = 0.35,
        scan_yaw_speed: int = 8,
        min_free_space_score: float = 0.22,
        timeout_action: str = "land",
        clearance_distance_cm: float = 70.0,
        bypass_forward_distance_cm: float = 120.0,
        bypass_forward_speed: int = 35,
        bypass_lateral_direction: str = "right",
        max_sidestep_seconds: float = 10.0,
    ) -> None:
        self.safety_manager = safety_manager
        # The following legacy keyword arguments remain accepted so external test
        # harnesses do not break; distance-only routing never reads them.
        del avoidance_yaw_speed, max_avoidance_seconds
        del forward_speed_in_caution_ratio, scan_yaw_speed, min_free_space_score
        self.avoidance_lateral_speed = self._non_negative_int(avoidance_lateral_speed, 0)
        self.recovery_clear_frames = self._positive_int(recovery_clear_frames, 10)
        self.detect_confirm_frames = self._positive_int(detect_confirm_frames, 1)
        configured_clear_frames = recovery_clear_frames if clear_confirm_frames is None else clear_confirm_frames
        self.clear_confirm_frames = self._positive_int(configured_clear_frames, 1)
        normalized_timeout = str(timeout_action).strip().lower()
        self.timeout_action = normalized_timeout if normalized_timeout in {"land", "hover"} else "land"
        self.clearance_distance_cm = self._positive_float(clearance_distance_cm, 70.0)
        self.bypass_forward_distance_cm = self._positive_float(bypass_forward_distance_cm, 120.0)
        self.bypass_forward_speed = self._positive_int(bypass_forward_speed, 35)
        self.bypass_lateral_direction = (
            "left" if str(bypass_lateral_direction).strip().lower() == "left" else "right"
        )
        self.max_sidestep_seconds = self._positive_float(max_sidestep_seconds, 10.0)
        self.state = "CLEAR"
        self._clear_frames = 0
        self._plan_counter = 0
        self._bypass_phase: Optional[str] = None
        self._phase_started_at: Optional[float] = None
        self._bypass_lateral_seconds = 0.0
        self._bypass_forward_seconds = 0.0

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
            avoidance_lateral_speed=cls._config_int(obstacle, "avoidance_lateral_speed", 12),
            recovery_clear_frames=cls._config_int(obstacle, "recovery_clear_frames", 10),
            detect_confirm_frames=cls._config_int(obstacle, "detect_confirm_frames", 3),
            clear_confirm_frames=cls._config_int(obstacle, "clear_confirm_frames", 5),
            timeout_action=str(obstacle.get("timeout_action", "land")),
            clearance_distance_cm=cls._config_float(
                obstacle, "front_tof_clear_distance_cm", 70.0
            ),
            bypass_forward_distance_cm=cls._config_float(
                obstacle, "bypass_forward_distance_cm", 120.0
            ),
            bypass_forward_speed=cls._config_int(obstacle, "bypass_forward_speed", 35),
            bypass_lateral_direction=str(obstacle.get("bypass_lateral_direction", "right")),
            max_sidestep_seconds=cls._config_float(
                obstacle, "bypass_max_sidestep_seconds", 10.0
            ),
        )

    def plan(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        obstacle_priority: bool = False,
    ) -> AvoidanceDecision:
        """Run/continue the bypass route independently of target tracking."""
        del obstacle_priority
        self._plan_counter += 1
        plan_id = str(self._plan_counter)
        if self._bypass_phase is not None:
            return self._plan_active_bypass(follow_command, obstacle_result, plan_id)
        if not obstacle_result.found:
            return self._plan_clear(follow_command, plan_id)

        self._clear_frames = 0
        if (
            obstacle_result.consecutive_found_frames
            and obstacle_result.consecutive_found_frames < self.detect_confirm_frames
        ):
            self.state = "BRAKING"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state=self.state,
                action="BRAKE",
                reason="blocked candidate needs temporal confirmation",
                confidence=obstacle_result.confidence,
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=True,
            )

        return self._start_bypass(follow_command, obstacle_result, plan_id)

    def reset(self) -> None:
        """Clear planner state for a new autonomous session."""
        self.state = "CLEAR"
        self._clear_frames = 0
        self._plan_counter = 0
        self._reset_bypass()

    def _plan_clear(self, follow_command: RCCommand, plan_id: str) -> AvoidanceDecision:
        if self.state in {"AVOIDING", "RECOVERING", "BRAKING"}:
            self._clear_frames += 1
            if self._clear_frames < max(self.recovery_clear_frames, self.clear_confirm_frames):
                self.state = "RECOVERING"
                return self._decision(
                    self._limited_command(0, 0, follow_command.up_down, follow_command.yaw),
                    state=self.state,
                    action="HOLD",
                    reason="recovering after obstacle",
                    confidence=1.0,
                    plan_id=plan_id,
                )

        self.state = "CLEAR"
        self._clear_frames = 0
        return self._decision(
            self._limit(follow_command),
            state=self.state,
            action="FOLLOW",
            reason="clear",
            confidence=1.0,
            plan_id=plan_id,
        )

    def _start_bypass(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
    ) -> AvoidanceDecision:
        self._bypass_phase = "SIDE_STEP_OUT"
        self._phase_started_at = monotonic()
        self._bypass_lateral_seconds = 0.0
        self._bypass_forward_seconds = 0.0
        self.state = "AVOIDING"
        return self._route_decision(
            follow_command, plan_id, "SIDE_STEP_OUT", "moving sideways until front ToF > 70 cm", obstacle_result
        )

    def _plan_active_bypass(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
    ) -> AvoidanceDecision:
        now = monotonic()
        phase_started = self._phase_started_at if self._phase_started_at is not None else now
        elapsed = max(0.0, now - phase_started)

        if self._bypass_phase == "FAILSAFE":
            return self._route_failsafe(follow_command, obstacle_result, plan_id)

        if self._bypass_phase == "SIDE_STEP_OUT":
            if self._front_is_clear(obstacle_result):
                self._bypass_lateral_seconds += elapsed
                self._bypass_phase = "FORWARD_BYPASS"
                self._phase_started_at = now
                return self._route_decision(
                    follow_command, plan_id, "FORWARD_120CM", "front clear; advancing 1.2 m", obstacle_result
                )
            if self._bypass_lateral_seconds + elapsed >= self.max_sidestep_seconds:
                return self._route_failsafe(follow_command, obstacle_result, plan_id)
            return self._route_decision(
                follow_command, plan_id, "SIDE_STEP_OUT", "moving sideways until front ToF > 70 cm", obstacle_result
            )

        if self._bypass_phase == "FORWARD_BYPASS":
            if not self._front_is_clear(obstacle_result):
                self._bypass_forward_seconds += elapsed
                self._bypass_phase = "SIDE_STEP_OUT"
                self._phase_started_at = now
                return self._route_decision(
                    follow_command, plan_id, "SIDE_STEP_OUT", "front distance fell to 70 cm; widening bypass", obstacle_result
                )
            forward_seconds = self._forward_duration_seconds()
            if self._bypass_forward_seconds + elapsed >= forward_seconds:
                self._bypass_forward_seconds = forward_seconds
                self._bypass_phase = "SIDE_STEP_RETURN"
                self._phase_started_at = now
                return self._route_decision(
                    follow_command, plan_id, "SIDE_STEP_RETURN", "returning by equal lateral time", obstacle_result
                )
            return self._route_decision(
                follow_command, plan_id, "FORWARD_120CM", "advancing 1.2 m", obstacle_result
            )

        if elapsed >= self._bypass_lateral_seconds:
            self._reset_bypass()
            self.state = "CLEAR"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state="CLEAR",
                action="BYPASS_COMPLETE",
                reason="bypass complete; lateral position restored by dead reckoning",
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=True,
            )
        return self._route_decision(
            follow_command, plan_id, "SIDE_STEP_RETURN", "returning by equal lateral time", obstacle_result
        )

    def _route_decision(
        self, follow_command: RCCommand, plan_id: str, action: str, reason: str,
        observation: Optional[ObstacleResult] = None,
    ) -> AvoidanceDecision:
        lateral_sign = 1 if self.bypass_lateral_direction == "right" else -1
        if action == "SIDE_STEP_OUT":
            command = self._limited_command(
                lateral_sign * self.avoidance_lateral_speed, 0, follow_command.up_down, 0
            )
        elif action == "FORWARD_120CM":
            command = self._limited_command(0, self.bypass_forward_speed, follow_command.up_down, 0)
        else:
            command = self._limited_command(
                -lateral_sign * self.avoidance_lateral_speed, 0, follow_command.up_down, 0
            )
        self.state = "AVOIDING"
        return self._decision(
            command,
            state=self.state,
            action=action,
            reason=reason,
            confidence=1.0,
            plan_id=plan_id,
            owns_motion=True,
            observation=observation,
        )

    def _route_failsafe(
        self, follow_command: RCCommand, obstacle_result: ObstacleResult, plan_id: str
    ) -> AvoidanceDecision:
        self._bypass_phase = "FAILSAFE"
        self.state = "FAILSAFE"
        return self._decision(
            self._limited_command(0, 0, follow_command.up_down, 0),
            state=self.state,
            action="LAND" if self.timeout_action == "land" else "HOVER",
            reason="front clearance did not exceed 70 cm within side-step safety limit",
            confidence=1.0,
            plan_id=plan_id,
            requires_landing=self.timeout_action == "land",
            observation=obstacle_result,
            owns_motion=True,
        )

    def _front_is_clear(self, result: ObstacleResult) -> bool:
        if result.front_distance_status == "out_of_range":
            return True
        return (
            result.front_distance_cm is not None
            and result.front_distance_cm > self.clearance_distance_cm
        )

    def _forward_duration_seconds(self) -> float:
        limited_speed = abs(self._limited_command(0, self.bypass_forward_speed, 0, 0).forward_backward)
        return self.bypass_forward_distance_cm / max(1, limited_speed)

    def _reset_bypass(self) -> None:
        self._bypass_phase = None
        self._phase_started_at = None
        self._bypass_lateral_seconds = 0.0
        self._bypass_forward_seconds = 0.0

    def _decision(self, command: RCCommand, **kwargs: object) -> AvoidanceDecision:
        return AvoidanceDecision(command=command, **kwargs)  # type: ignore[arg-type]

    def _limit(self, command: RCCommand) -> RCCommand:
        return self._limited_command(*command.as_tuple())

    def _limited_command(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> RCCommand:
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
