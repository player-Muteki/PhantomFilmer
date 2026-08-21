"""Target-loss recovery dedicated to side-follow mode."""

import math
from dataclasses import dataclass

from control.follow_control import RCCommand


@dataclass(frozen=True)
class SideFollowRecoveryConfig:
    """Bounded motion settings used after a side-follow target exits frame."""

    lateral_search_seconds: float = 3.0
    lateral_search_speed: int = 20
    rotation_yaw_speed: int = 30
    rotation_degrees: float = 360.0
    rotation_fallback_seconds: float = 12.0
    final_hover_seconds: float = 3.0

    @classmethod
    def from_config(cls, config: dict[str, object]) -> "SideFollowRecoveryConfig":
        side = config.get("side_follow", {}) if isinstance(config, dict) else {}
        search = config.get("target_search", {}) if isinstance(config, dict) else {}
        side_cfg = side if isinstance(side, dict) else {}
        search_cfg = search if isinstance(search, dict) else {}

        yaw_speed = cls._positive_int(
            side_cfg.get("lost_rotation_yaw_speed"),
            cls._positive_int(search_cfg.get("yaw_speed"), 30),
        )
        rotation_degrees = cls._positive_float(
            side_cfg.get("lost_rotation_degrees"),
            cls._positive_float(search_cfg.get("full_rotation_degrees"), 360.0),
        )
        return cls(
            lateral_search_seconds=cls._positive_float(
                side_cfg.get("lost_lateral_search_seconds"), 3.0
            ),
            lateral_search_speed=cls._positive_int(
                side_cfg.get("lost_lateral_search_speed"), 20
            ),
            rotation_yaw_speed=yaw_speed,
            rotation_degrees=rotation_degrees,
            rotation_fallback_seconds=cls._positive_float(
                side_cfg.get("lost_rotation_fallback_seconds"),
                cls._positive_float(
                    search_cfg.get("full_rotation_fallback_seconds"),
                    rotation_degrees / max(1, yaw_speed),
                ),
            ),
            final_hover_seconds=cls._positive_float(
                side_cfg.get("lost_final_hover_seconds"), 3.0
            ),
        )

    @staticmethod
    def _positive_float(value: object, default: float) -> float:
        if not isinstance(value, (int, float, str)):
            return default
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = default
        if not math.isfinite(parsed):
            parsed = default
        return max(0.01, parsed)

    @staticmethod
    def _positive_int(value: object, default: int) -> int:
        if not isinstance(value, (int, float, str)):
            return default
        try:
            parsed = int(value)
        except (OverflowError, TypeError, ValueError):
            parsed = default
        return max(1, parsed)


@dataclass(frozen=True)
class SideFollowRecoveryDecision:
    """One loss-recovery output consumed by the flight session."""

    command: RCCommand
    state: str
    reason: str
    requires_landing: bool = False


@dataclass
class SideFollowRecoveryDebug:
    """Inspectable recovery state for flight logs and tests."""

    state: str = "IDLE"
    horizontal_direction: int = 0
    elapsed_seconds: float = 0.0
    rotation_progress_degrees: float = 0.0
    yaw_telemetry_available: bool = False


class SideFollowRecoveryController:
    """Move toward a frame exit, rotate once, then hover and land."""

    def __init__(self, config: SideFollowRecoveryConfig) -> None:
        self.config = config
        self.state = "IDLE"
        self.phase_started_at: float | None = None
        self.has_observed_target = False
        self.last_horizontal_direction = 0
        self.rotation_progress_degrees = 0.0
        self.rotation_last_yaw: float | None = None
        self.last_debug = SideFollowRecoveryDebug()

    @classmethod
    def from_config(cls, config: dict[str, object]) -> "SideFollowRecoveryController":
        return cls(SideFollowRecoveryConfig.from_config(config))

    @property
    def recovering(self) -> bool:
        return self.state != "IDLE"

    def reset(self, *, preserve_direction: bool = False) -> None:
        """Clear an active recovery; optionally retain the last exit direction."""
        self.state = "IDLE"
        self.phase_started_at = None
        self.rotation_progress_degrees = 0.0
        self.rotation_last_yaw = None
        if not preserve_direction:
            self.has_observed_target = False
            self.last_horizontal_direction = 0
        self.last_debug = SideFollowRecoveryDebug(
            horizontal_direction=self.last_horizontal_direction
        )

    def observe_target(
        self,
        target_result: dict[str, object],
        frame_width: int,
    ) -> None:
        """Remember which side of the image a fresh target most recently occupied."""
        if not self._fresh_target(target_result):
            return
        center = target_result.get("center")
        if not isinstance(center, (tuple, list)) or len(center) != 2:
            return
        try:
            center_x = float(center[0])
        except (TypeError, ValueError):
            return
        error = center_x - frame_width / 2.0
        if error != 0:
            self.last_horizontal_direction = 1 if error > 0 else -1
        self.has_observed_target = True

    def update(
        self,
        target_result: dict[str, object],
        frame_width: int,
        now: float,
        yaw_deg: int | None,
    ) -> SideFollowRecoveryDecision | None:
        """Return a recovery command, or ``None`` when normal following may resume."""
        if self._fresh_target(target_result):
            self.observe_target(target_result, frame_width)
            self.reset(preserve_direction=True)
            return None

        if self.state == "IDLE":
            if self.has_observed_target and self.last_horizontal_direction != 0:
                self._enter("SIDE_LOST_LATERAL", now)
            else:
                self._enter("SIDE_LOST_FINAL_HOVER", now)

        elapsed = now - (
            self.phase_started_at if self.phase_started_at is not None else now
        )
        direction = self.last_horizontal_direction

        if self.state == "SIDE_LOST_LATERAL":
            if elapsed < self.config.lateral_search_seconds:
                return self._decision(
                    RCCommand(left_right=self.config.lateral_search_speed * direction),
                    elapsed,
                    "move laterally toward the target's frame-exit direction",
                )
            self._enter("SIDE_LOST_ROTATING", now)
            self._reset_rotation_tracking()
            elapsed = 0.0

        if self.state == "SIDE_LOST_ROTATING":
            telemetry_available = self._update_rotation_progress(yaw_deg, direction)
            complete = (
                self.rotation_progress_degrees >= self.config.rotation_degrees
                if telemetry_available
                else elapsed >= self.config.rotation_fallback_seconds
            )
            if not complete:
                progress = (
                    f"{self.rotation_progress_degrees:.0f}/"
                    f"{self.config.rotation_degrees:.0f} deg"
                    if telemetry_available
                    else f"fallback {elapsed:.1f}/"
                    f"{self.config.rotation_fallback_seconds:.1f} s"
                )
                return self._decision(
                    RCCommand(yaw=self.config.rotation_yaw_speed * direction),
                    elapsed,
                    f"rotate toward frame-exit direction; {progress}",
                    yaw_telemetry_available=telemetry_available,
                )
            self._enter("SIDE_LOST_FINAL_HOVER", now)
            elapsed = 0.0

        if self.state == "SIDE_LOST_FINAL_HOVER":
            if elapsed < self.config.final_hover_seconds:
                return self._decision(
                    RCCommand(), elapsed, "full rotation complete; final hover"
                )
            self._enter("SIDE_LOST_LANDING", now)

        return self._decision(
            RCCommand(),
            0.0,
            "side-follow recovery exhausted",
            requires_landing=True,
        )

    def _update_rotation_progress(self, yaw_deg: int | None, direction: int) -> bool:
        """Accumulate only yaw changes in the commanded direction across wrap."""
        if yaw_deg is None:
            self.rotation_last_yaw = None
            return False
        current = float(yaw_deg)
        if self.rotation_last_yaw is None:
            self.rotation_last_yaw = current
            return True
        delta = ((current - self.rotation_last_yaw + 180.0) % 360.0) - 180.0
        self.rotation_last_yaw = current
        commanded_delta = delta * direction
        if commanded_delta > 0:
            self.rotation_progress_degrees += commanded_delta
        return True

    def _reset_rotation_tracking(self) -> None:
        self.rotation_progress_degrees = 0.0
        self.rotation_last_yaw = None

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self.phase_started_at = now

    def _decision(
        self,
        command: RCCommand,
        elapsed: float,
        reason: str,
        *,
        requires_landing: bool = False,
        yaw_telemetry_available: bool = False,
    ) -> SideFollowRecoveryDecision:
        self.last_debug = SideFollowRecoveryDebug(
            state=self.state,
            horizontal_direction=self.last_horizontal_direction,
            elapsed_seconds=max(0.0, elapsed),
            rotation_progress_degrees=self.rotation_progress_degrees,
            yaw_telemetry_available=yaw_telemetry_available,
        )
        return SideFollowRecoveryDecision(
            command=command,
            state=self.state,
            reason=reason,
            requires_landing=requires_landing,
        )

    @staticmethod
    def _fresh_target(target_result: dict[str, object]) -> bool:
        return (
            bool(target_result.get("found"))
            and not bool(target_result.get("is_predicted"))
            and not bool(target_result.get("ambiguous"))
        )
