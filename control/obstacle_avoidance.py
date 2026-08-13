"""Deterministic lateral-only avoidance for front-ToF observations."""

from dataclasses import dataclass, field
from math import inf
from statistics import median
from time import monotonic
from typing import Dict, List, Optional

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
        bypass_lateral_distance_cm: float = 100.0,
        max_sidestep_seconds: float = 10.0,
        lost_tof_sample_count: int = 5,
        lost_clearance_margin_cm: float = 30.0,
        lost_forward_margin_cm: float = 20.0,
        lost_clear_confirm_samples: int = 3,
        forward_pulse_seconds: float = 0.30,
        forward_check_seconds: float = 0.20,
        post_bypass_turn_degrees: float = 90.0,
        post_bypass_turn_speed: int = 12,
        post_bypass_turn_timeout_seconds: float = 15.0,
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
        # Retain the old constructor parameters for source compatibility only.
        # Avoidance is lateral/yaw-only and never emits forward motion.
        del bypass_forward_distance_cm, bypass_forward_speed
        self.bypass_lateral_direction = (
            "left" if str(bypass_lateral_direction).strip().lower() == "left" else "right"
        )
        self.bypass_lateral_distance_cm = self._positive_float(
            bypass_lateral_distance_cm, 100.0
        )
        self.max_sidestep_seconds = self._positive_float(max_sidestep_seconds, 10.0)
        self.lost_tof_sample_count = self._positive_int(lost_tof_sample_count, 5)
        self.lost_clearance_margin_cm = self._positive_float(lost_clearance_margin_cm, 30.0)
        # Kept as accepted compatibility arguments for older callers.  The
        # target-loss route no longer advances after its right sidestep.
        del lost_forward_margin_cm, forward_pulse_seconds, forward_check_seconds
        self.lost_clear_confirm_samples = self._positive_int(lost_clear_confirm_samples, 3)
        self.post_bypass_turn_degrees = self._positive_float(post_bypass_turn_degrees, 90.0)
        self.post_bypass_turn_speed = self._positive_int(post_bypass_turn_speed, 12)
        self.post_bypass_turn_timeout_seconds = self._positive_float(
            post_bypass_turn_timeout_seconds, 15.0
        )
        self.state = "CLEAR"
        self._clear_frames = 0
        self._plan_counter = 0
        self._bypass_phase: Optional[str] = None
        self._phase_started_at: Optional[float] = None
        self._bypass_lateral_seconds = 0.0
        self._dynamic_lost_bypass = False
        self._dynamic_reference_cm: Optional[float] = None
        self._side_clear_samples = 0
        self._last_front_sequence = -1
        self._lost_episode_id: Optional[int] = None
        self._lost_probe_complete = False
        self._lost_probe_samples: List[Optional[float]] = []
        self._turn_progress_degrees = 0.0
        self._turn_last_yaw: Optional[float] = None

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
            bypass_lateral_direction=str(obstacle.get("bypass_lateral_direction", "right")),
            bypass_lateral_distance_cm=cls._config_float(
                obstacle, "bypass_lateral_distance_cm", 100.0
            ),
            max_sidestep_seconds=cls._config_float(
                obstacle, "bypass_max_sidestep_seconds", 10.0
            ),
            lost_tof_sample_count=cls._config_int(
                obstacle, "lost_tof_sample_count", 5
            ),
            lost_clearance_margin_cm=cls._config_float(
                obstacle, "lost_clearance_margin_cm", 30.0
            ),
            lost_forward_margin_cm=20.0,
            lost_clear_confirm_samples=cls._config_int(
                obstacle, "lost_clear_confirm_samples", 3
            ),
            forward_pulse_seconds=0.30,
            forward_check_seconds=0.20,
            post_bypass_turn_degrees=cls._config_float(
                obstacle, "post_bypass_turn_degrees", 90.0
            ),
            post_bypass_turn_speed=cls._config_int(
                obstacle, "post_bypass_turn_speed", 12
            ),
            post_bypass_turn_timeout_seconds=cls._config_float(
                obstacle, "post_bypass_turn_timeout_seconds", 15.0
            ),
        )

    def plan(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        obstacle_priority: bool = False,
        lost_episode_id: Optional[int] = None,
        yaw_deg: Optional[int] = None,
    ) -> AvoidanceDecision:
        """Run/continue the bypass route independently of target tracking."""
        self._plan_counter += 1
        plan_id = str(self._plan_counter)
        if self._bypass_phase is not None:
            return self._plan_active_bypass(
                follow_command, obstacle_result, plan_id, yaw_deg=yaw_deg
            )
        if obstacle_priority and lost_episode_id is not None:
            if lost_episode_id != self._lost_episode_id:
                self._begin_lost_probe(lost_episode_id, obstacle_result)
            if not self._lost_probe_complete:
                return self._plan_lost_probe(follow_command, obstacle_result, plan_id)
        elif not obstacle_priority:
            self._cancel_lost_probe()
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
        self._cancel_lost_probe()

    def cancel_lost_target_recovery(self) -> None:
        """Stop only the target-loss route when ReID freshly reacquires the target."""
        # Once a right sidestep starts, finish the configured one metre before
        # returning control. This avoids a brief ReID hit producing an arbitrary
        # lateral distance. Reacquisition may still cancel the later turn.
        if self._dynamic_lost_bypass and self._bypass_phase == "SIDE_STEP_OUT":
            return
        if self._dynamic_lost_bypass or self._lost_episode_id is not None:
            self._reset_bypass()
            self._cancel_lost_probe()
            self.state = "CLEAR"
            self._clear_frames = 0

    def _begin_lost_probe(
        self, episode_id: int, obstacle_result: ObstacleResult
    ) -> None:
        self._lost_episode_id = episode_id
        self._lost_probe_complete = False
        self._lost_probe_samples = []
        # The first lost tick establishes the cache baseline. Only later ToF
        # sequence numbers count as the requested five fresh measurements.
        self._last_front_sequence = obstacle_result.front_distance_sequence

    def _cancel_lost_probe(self) -> None:
        self._lost_episode_id = None
        self._lost_probe_complete = False
        self._lost_probe_samples = []

    def _plan_lost_probe(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
    ) -> AvoidanceDecision:
        sequence = obstacle_result.front_distance_sequence
        if sequence != self._last_front_sequence:
            self._last_front_sequence = sequence
            sample = (
                obstacle_result.front_distance_cm
                if obstacle_result.front_distance_status == "valid"
                else None
            )
            self._lost_probe_samples.append(sample)

        count = len(self._lost_probe_samples)
        if count < self.lost_tof_sample_count:
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state="IR_OCCLUSION_CHECK",
                action="HOVER",
                reason=f"collecting fresh front ToF samples {count}/{self.lost_tof_sample_count}",
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=True,
            )

        values = [inf if value is None else float(value) for value in self._lost_probe_samples]
        reference = float(median(values))
        self._lost_probe_complete = True
        if reference > 120.0:
            self.state = "CLEAR"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state="CLEAR",
                action="SEARCH_RELEASE",
                reason="target lost; median front ToF is out of range",
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=False,
            )

        self._dynamic_lost_bypass = True
        self._dynamic_reference_cm = reference
        return self._start_bypass(follow_command, obstacle_result, plan_id)

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
        self._side_clear_samples = 0
        self._last_front_sequence = obstacle_result.front_distance_sequence
        self.state = "AVOIDING"
        return self._route_decision(
            follow_command,
            plan_id,
            "SIDE_STEP_OUT",
            self._side_step_reason(),
            obstacle_result,
        )

    def _plan_active_bypass(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
        yaw_deg: Optional[int] = None,
    ) -> AvoidanceDecision:
        now = monotonic()
        phase_started = self._phase_started_at if self._phase_started_at is not None else now
        elapsed = max(0.0, now - phase_started)

        if self._bypass_phase == "FAILSAFE":
            return self._route_failsafe(follow_command, obstacle_result, plan_id)

        if self._bypass_phase == "POST_BYPASS_LEFT_TURN":
            return self._plan_post_bypass_left_turn(
                follow_command, obstacle_result, plan_id, yaw_deg, elapsed
            )

        if self._bypass_phase == "SIDE_STEP_OUT":
            if elapsed >= self._sidestep_duration_seconds():
                self._bypass_lateral_seconds += elapsed
                self._bypass_phase = "POST_BYPASS_LEFT_TURN"
                self._phase_started_at = now
                self._turn_progress_degrees = 0.0
                self._turn_last_yaw = (
                    float(yaw_deg) if yaw_deg is not None else None
                )
                return self._decision(
                    self._limited_command(
                        0, 0, follow_command.up_down, -self.post_bypass_turn_speed
                    ),
                    state="AVOIDING",
                    action="POST_BYPASS_LEFT_TURN",
                    reason=(
                        "right sidestep exposed the target area; "
                        f"turning left {self.post_bypass_turn_degrees:.0f} degrees to search"
                    ),
                    plan_id=plan_id,
                    observation=obstacle_result,
                    owns_motion=True,
                )
            if elapsed >= self.max_sidestep_seconds:
                return self._route_failsafe(follow_command, obstacle_result, plan_id)
            return self._route_decision(
                follow_command, plan_id, "SIDE_STEP_OUT", self._side_step_reason(), obstacle_result
            )

        return self._route_failsafe(follow_command, obstacle_result, plan_id)

    def _plan_post_bypass_left_turn(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
        yaw_deg: Optional[int],
        elapsed: float,
    ) -> AvoidanceDecision:
        if elapsed >= self.post_bypass_turn_timeout_seconds:
            return self._route_failsafe(follow_command, obstacle_result, plan_id)
        if yaw_deg is None:
            self._turn_last_yaw = None
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state="AVOIDING",
                action="POST_BYPASS_TURN_WAIT",
                reason="waiting for yaw telemetry before 90-degree left turn",
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=True,
            )
        current = float(yaw_deg)
        if self._turn_last_yaw is None:
            self._turn_last_yaw = current
        else:
            delta = ((current - self._turn_last_yaw + 180.0) % 360.0) - 180.0
            self._turn_last_yaw = current
            if delta < 0:
                self._turn_progress_degrees += -delta
        if self._turn_progress_degrees >= self.post_bypass_turn_degrees:
            self._reset_bypass()
            self.state = "CLEAR"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state="CLEAR",
                action="BYPASS_COMPLETE",
                reason="left search turn reached 90 degrees; releasing follow/search arbitration",
                plan_id=plan_id,
                observation=obstacle_result,
                owns_motion=True,
            )
        return self._decision(
            self._limited_command(0, 0, follow_command.up_down, -self.post_bypass_turn_speed),
            state="AVOIDING",
            action="POST_BYPASS_LEFT_TURN",
            reason=(
                f"turning left at yaw speed {self.post_bypass_turn_speed}: "
                f"{self._turn_progress_degrees:.1f}/{self.post_bypass_turn_degrees:.1f} degrees"
            ),
            plan_id=plan_id,
            observation=obstacle_result,
            owns_motion=True,
        )

    def _route_decision(
        self, follow_command: RCCommand, plan_id: str, action: str, reason: str,
        observation: Optional[ObstacleResult] = None,
    ) -> AvoidanceDecision:
        lateral_sign = 1 if self.bypass_lateral_direction == "right" else -1
        command = self._limited_command(
            lateral_sign * self.avoidance_lateral_speed, 0, follow_command.up_down, 0
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
            reason="obstacle-recovery maneuver did not complete within its safety limit",
            confidence=1.0,
            plan_id=plan_id,
            requires_landing=self.timeout_action == "land",
            observation=obstacle_result,
            owns_motion=True,
        )

    def _side_step_reason(self) -> str:
        return (
            f"moving right for an estimated {self.bypass_lateral_distance_cm:.0f} cm "
            f"at RC speed {self.avoidance_lateral_speed}"
        )

    def _sidestep_duration_seconds(self) -> float:
        limited_speed = abs(
            self._limited_command(self.avoidance_lateral_speed, 0, 0, 0).left_right
        )
        return self.bypass_lateral_distance_cm / max(1, limited_speed)

    def _reset_bypass(self) -> None:
        self._bypass_phase = None
        self._phase_started_at = None
        self._bypass_lateral_seconds = 0.0
        self._dynamic_lost_bypass = False
        self._dynamic_reference_cm = None
        self._side_clear_samples = 0
        self._turn_progress_degrees = 0.0
        self._turn_last_yaw = None

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
