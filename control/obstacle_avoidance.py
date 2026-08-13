"""Deterministic local route planning for visual obstacle observations."""

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
        maximum_forward_speed_in_caution: int = 20,
        scan_yaw_speed: int = 8,
        min_free_space_score: float = 0.22,
        timeout_action: str = "land",
    ) -> None:
        self.safety_manager = safety_manager
        self.avoidance_yaw_speed = self._non_negative_int(avoidance_yaw_speed, 18)
        self.avoidance_lateral_speed = self._non_negative_int(avoidance_lateral_speed, 0)
        self.max_avoidance_seconds = self._positive_float(max_avoidance_seconds, 5.0)
        self.recovery_clear_frames = self._positive_int(recovery_clear_frames, 10)
        self.detect_confirm_frames = self._positive_int(detect_confirm_frames, 1)
        configured_clear_frames = recovery_clear_frames if clear_confirm_frames is None else clear_confirm_frames
        self.clear_confirm_frames = self._positive_int(configured_clear_frames, 1)
        self.maximum_forward_speed_in_caution = self._non_negative_int(
            maximum_forward_speed_in_caution, 20
        )
        self.scan_yaw_speed = self._non_negative_int(scan_yaw_speed, 8)
        self.min_free_space_score = self._clamp_float(min_free_space_score, 0.0, 1.0, 0.22)
        normalized_timeout = str(timeout_action).strip().lower()
        self.timeout_action = normalized_timeout if normalized_timeout in {"land", "hover"} else "land"
        self.state = "CLEAR"
        self._clear_frames = 0
        self._avoidance_started_at: Optional[float] = None
        self._last_direction = "right"
        self._plan_counter = 0

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
            avoidance_lateral_speed=cls._config_int(obstacle, "avoidance_lateral_speed", 12),
            max_avoidance_seconds=cls._config_float(obstacle, "max_avoidance_seconds", 5.0),
            recovery_clear_frames=cls._config_int(obstacle, "recovery_clear_frames", 10),
            detect_confirm_frames=cls._config_int(obstacle, "detect_confirm_frames", 3),
            clear_confirm_frames=cls._config_int(obstacle, "clear_confirm_frames", 5),
            maximum_forward_speed_in_caution=cls._config_int(
                obstacle, "maximum_forward_speed_in_caution", 20
            ),
            scan_yaw_speed=cls._config_int(obstacle, "scan_yaw_speed", 8),
            min_free_space_score=cls._config_float(obstacle, "min_free_space_score", 0.22),
            timeout_action=str(obstacle.get("timeout_action", "land")),
        )

    def plan(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        obstacle_priority: bool = False,
    ) -> AvoidanceDecision:
        """Return one bounded action after applying obstacle priority rules.

        obstacle_priority 用于目标丢失场景：即使期望指令全零，也要对已确认的
        阻挡障碍主动绕行，而不是只刹车悬停（后者会让"被障碍挡住的目标"期间
        原地等待，直到超时降落）。障碍存在时仍先做时间确认再绕行，前进恒为 0。
        """
        self._plan_counter += 1
        plan_id = str(self._plan_counter)
        if not obstacle_result.found:
            return self._plan_clear(follow_command, plan_id)

        self._clear_frames = 0
        if obstacle_result.side in ("left", "right"):
            self._last_direction = "right" if obstacle_result.side == "left" else "left"

        if not obstacle_priority and not any(follow_command.as_tuple()):
            self.state = "BRAKING"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state=self.state,
                action="BRAKE",
                reason="obstacle detected while command is already stationary",
                confidence=obstacle_result.confidence,
                plan_id=plan_id,
                observation=obstacle_result,
            )

        if obstacle_result.state == "CAUTION":
            self.state = "CAUTION"
            return self._decision(
                self._caution_command(follow_command),
                state=self.state,
                action="SLOW_FOLLOW",
                reason="obstacle caution",
                confidence=obstacle_result.confidence,
                plan_id=plan_id,
                observation=obstacle_result,
            )

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
            )

        return self._plan_blocked(follow_command, obstacle_result, plan_id)

    def reset(self) -> None:
        """Clear planner state for a new autonomous session."""
        self.state = "CLEAR"
        self._clear_frames = 0
        self._avoidance_started_at = None
        self._last_direction = "right"
        self._plan_counter = 0

    def _plan_clear(self, follow_command: RCCommand, plan_id: str) -> AvoidanceDecision:
        if self.state in {"BLOCKED", "AVOIDING", "RECOVERING", "CAUTION", "SCAN", "BRAKING"}:
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
        self._avoidance_started_at = None
        return self._decision(
            self._limit(follow_command),
            state=self.state,
            action="FOLLOW",
            reason="clear",
            confidence=1.0,
            plan_id=plan_id,
        )

    def _plan_blocked(
        self,
        follow_command: RCCommand,
        obstacle_result: ObstacleResult,
        plan_id: str,
    ) -> AvoidanceDecision:
        now = monotonic()
        if self._avoidance_started_at is None:
            self._avoidance_started_at = now
        elapsed = now - self._avoidance_started_at
        if elapsed > self.max_avoidance_seconds:
            self.state = "FAILSAFE"
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, 0),
                state=self.state,
                action="LAND" if self.timeout_action == "land" else "HOVER",
                reason="avoidance timeout; no safe local route confirmed",
                confidence=obstacle_result.confidence,
                plan_id=plan_id,
                requires_landing=self.timeout_action == "land",
                observation=obstacle_result,
            )

        direction, direction_score, used_free_space = self._choose_direction(obstacle_result)
        if used_free_space and direction_score < self.min_free_space_score:
            self.state = "SCAN"
            direction = self._last_direction
            yaw = self.scan_yaw_speed if direction == "right" else -self.scan_yaw_speed
            return self._decision(
                self._limited_command(0, 0, follow_command.up_down, yaw),
                state=self.state,
                action="SCAN_RIGHT" if direction == "right" else "SCAN_LEFT",
                reason="all local sectors are uncertain or blocked",
                confidence=obstacle_result.confidence,
                plan_id=plan_id,
                observation=obstacle_result,
            )

        self.state = "AVOIDING"
        lateral = self.avoidance_lateral_speed if direction == "right" else -self.avoidance_lateral_speed
        yaw = self.avoidance_yaw_speed if direction == "right" else -self.avoidance_yaw_speed
        return self._decision(
            self._limited_command(lateral, 0, follow_command.up_down, yaw),
            state=self.state,
            action="DETOUR_RIGHT" if direction == "right" else "DETOUR_LEFT",
            reason=f"avoiding {direction}; free-space score={direction_score:.2f}",
            confidence=obstacle_result.confidence,
            plan_id=plan_id,
            observation=obstacle_result,
        )

    def _choose_direction(self, result: ObstacleResult) -> tuple[str, float, bool]:
        if result.free_space:
            left_values = [value for key, value in result.free_space.items() if key in {"far_left", "left"}]
            right_values = [value for key, value in result.free_space.items() if key in {"right", "far_right"}]
            if not left_values and not right_values:
                # 通用命名（sector_0..sector_{N-1}）按数值索引对称分半：
                # 左半、右半各取一半扇区，奇数时中间扇区不参与两侧评分。
                indexed = sorted(
                    (int("".join(ch for ch in key if ch.isdigit())), value)
                    for key, value in result.free_space.items()
                    if any(ch.isdigit() for ch in key)
                )
                if indexed:
                    count = len(indexed)
                    left_values = [value for index, value in indexed if index < count // 2]
                    right_values = [value for index, value in indexed if index >= count - count // 2]
            left_score = sum(left_values) / len(left_values) if left_values else 0.0
            right_score = sum(right_values) / len(right_values) if right_values else 0.0
            if right_score > left_score:
                direction = "right"
            elif left_score > right_score:
                direction = "left"
            else:
                direction = self._last_direction
            self._last_direction = direction
            return direction, max(left_score, right_score), True
        direction = self._avoidance_direction(result.side)
        return direction, 1.0, False

    def _caution_command(self, follow_command: RCCommand) -> RCCommand:
        """Cap positive forward motion at the configured CAUTION speed."""
        forward = follow_command.forward_backward
        if forward > 0:
            forward = min(forward, self.maximum_forward_speed_in_caution)
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
