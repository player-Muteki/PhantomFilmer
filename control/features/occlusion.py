"""Occlusion-aware target recovery for visual follow missions.

This feature deliberately separates three different situations that used to be
treated as the same "target lost + obstacle" event:

* the mission has never established a trustworthy target lock;
* a previously locked target disappeared without evidence of an occluder;
* a persistent blocked obstacle overlaps the predicted target position.

Only the third case may command a bounded lateral peek.  The feature never
commands forward/backward, vertical, and yaw motion during that peek.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Optional, Tuple

from control.follow_control import RCCommand
from control.kernel.features import ArbitrationContext, FeatureProposal
from vision.obstacle_detect import ObstacleResult


Box = Tuple[int, int, int, int]


@dataclass(frozen=True)
class TargetMotionSample:
    """One trustworthy target position retained for field-of-view exit checks."""

    timestamp: float
    center_x: float
    center_y: float
    bbox: Box


@dataclass(frozen=True)
class ExitAssessment:
    """Classification result produced before obstacle-linked recovery is allowed."""

    classification: str
    reason: str


@dataclass(frozen=True)
class OcclusionRecoveryConfig:
    """Validated thresholds for the target-memory recovery state machine."""

    enabled: bool = True
    initial_lock_frames: int = 10
    initial_scan_yaw_speed: int = 20
    initial_scan_degrees: float = 360.0
    initial_scan_fallback_seconds: float = 18.0
    initial_acquire_timeout_seconds: float = 35.0
    occlusion_check_seconds: float = 1.5
    occlusion_max_age_seconds: float = 2.5
    occluder_min_area_ratio: float = 0.02
    occlusion_overlap_ratio: float = 0.25
    occluder_iou_threshold: float = 0.15
    occlusion_confirm_frames: int = 3
    unknown_loss_hover_seconds: float = 0.8
    exit_edge_margin_ratio: float = 0.15
    exit_history_frames: int = 8
    exit_min_displacement_ratio: float = 0.02
    exit_min_velocity_ratio_per_second: float = 0.15
    exit_time_to_edge_seconds: float = 1.2
    exit_yaw_saturation_ratio: float = 0.85
    maximum_yaw_speed: int = 25
    no_safe_route_seconds: float = 3.0
    prediction_margin_ratio: float = 0.20
    lateral_speed: int = 25
    lateral_pulse_seconds: float = 4.0
    settle_seconds: float = 0.55
    max_lateral_pulses: int = 1
    local_scan_degrees: float = 30.0
    local_scan_yaw_speed: int = 20
    local_scan_hold_seconds: float = 0.35
    local_scan_fallback_seconds: float = 1.5
    local_scan_return_tolerance_degrees: float = 3.0
    reacquire_frames: int = 5
    min_free_space_score: float = 0.35

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "OcclusionRecoveryConfig":
        raw = config.get("occlusion_recovery", {}) if isinstance(config, dict) else {}
        cfg = raw if isinstance(raw, dict) else {}
        return cls(
            enabled=bool(cfg.get("enabled", True)),
            initial_lock_frames=cls._positive_int(cfg.get("initial_lock_frames"), 10),
            initial_scan_yaw_speed=cls._non_negative_int(
                cfg.get("initial_scan_yaw_speed"), 20
            ),
            initial_scan_degrees=cls._positive_float(
                cfg.get("initial_scan_degrees"), 360.0
            ),
            initial_scan_fallback_seconds=cls._positive_float(
                cfg.get("initial_scan_fallback_seconds"), 18.0
            ),
            initial_acquire_timeout_seconds=cls._positive_float(
                cfg.get("initial_acquire_timeout_seconds"), 35.0
            ),
            occlusion_check_seconds=cls._positive_float(
                cfg.get("occlusion_check_seconds"), 1.5
            ),
            occlusion_max_age_seconds=cls._positive_float(
                cfg.get("occlusion_max_age_seconds"), 2.5
            ),
            occluder_min_area_ratio=cls._ratio(
                cfg.get("occluder_min_area_ratio"), 0.02
            ),
            occlusion_overlap_ratio=cls._ratio(
                cfg.get("occlusion_overlap_ratio"), 0.25
            ),
            occluder_iou_threshold=cls._ratio(
                cfg.get("occluder_iou_threshold"), 0.15
            ),
            occlusion_confirm_frames=cls._positive_int(
                cfg.get("occlusion_confirm_frames"), 3
            ),
            unknown_loss_hover_seconds=cls._positive_float(
                cfg.get("unknown_loss_hover_seconds"), 0.8
            ),
            exit_edge_margin_ratio=cls._ratio(
                cfg.get("exit_edge_margin_ratio"), 0.15
            ),
            exit_history_frames=cls._positive_int(
                cfg.get("exit_history_frames"), 8
            ),
            exit_min_displacement_ratio=cls._ratio(
                cfg.get("exit_min_displacement_ratio"), 0.02
            ),
            exit_min_velocity_ratio_per_second=cls._positive_float(
                cfg.get("exit_min_velocity_ratio_per_second"), 0.15
            ),
            exit_time_to_edge_seconds=cls._positive_float(
                cfg.get("exit_time_to_edge_seconds"), 1.2
            ),
            exit_yaw_saturation_ratio=cls._ratio(
                cfg.get("exit_yaw_saturation_ratio"), 0.85
            ),
            maximum_yaw_speed=cls._positive_int(
                config.get("maximum_yaw_speed") if isinstance(config, dict) else None,
                25,
            ),
            no_safe_route_seconds=cls._positive_float(
                cfg.get("no_safe_route_seconds"), 3.0
            ),
            prediction_margin_ratio=cls._ratio(
                cfg.get("prediction_margin_ratio"), 0.20
            ),
            lateral_speed=cls._non_negative_int(cfg.get("lateral_speed"), 25),
            lateral_pulse_seconds=cls._positive_float(
                cfg.get("lateral_pulse_seconds"), 4.0
            ),
            settle_seconds=cls._positive_float(cfg.get("settle_seconds"), 0.55),
            max_lateral_pulses=cls._positive_int(
                cfg.get("max_lateral_pulses"), 1
            ),
            local_scan_degrees=cls._positive_float(
                cfg.get("local_scan_degrees"), 30.0
            ),
            local_scan_yaw_speed=cls._non_negative_int(
                cfg.get("local_scan_yaw_speed"), 20
            ),
            local_scan_hold_seconds=cls._positive_float(
                cfg.get("local_scan_hold_seconds"), 0.35
            ),
            local_scan_fallback_seconds=cls._positive_float(
                cfg.get("local_scan_fallback_seconds"), 1.5
            ),
            local_scan_return_tolerance_degrees=cls._positive_float(
                cfg.get("local_scan_return_tolerance_degrees"), 3.0
            ),
            reacquire_frames=cls._positive_int(cfg.get("reacquire_frames"), 5),
            min_free_space_score=cls._ratio(
                cfg.get("min_free_space_score"), 0.35
            ),
        )

    @staticmethod
    def _positive_int(value: object, default: int) -> int:
        try:
            return max(1, int(value))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _non_negative_int(value: object, default: int) -> int:
        try:
            return max(0, int(value))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _positive_float(value: object, default: float) -> float:
        try:
            return max(0.01, float(value))
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _ratio(value: object, default: float) -> float:
        try:
            return max(0.0, min(1.0, float(value)))
        except (TypeError, ValueError):
            return default


class OcclusionRecoveryFeature:
    """Remember a locked target and own only bounded occlusion-recovery motion."""

    feature_name = "occlusion_recovery"
    _RECOVERY_STATES = {
        "OCCLUSION_CHECK",
        "OCCLUSION_NO_SAFE_ROUTE",
        "OCCLUSION_BYPASS",
        "OCCLUSION_SETTLE",
        "OCCLUSION_LOCAL_SCAN_OUT",
        "OCCLUSION_LOCAL_SCAN_HOLD",
        "OCCLUSION_LOCAL_SCAN_RETURN",
        "LOSS_UNCERTAIN_HOLD",
        "REACQUIRE_VERIFY",
        "FALLBACK_SEARCH",
    }

    def __init__(
        self,
        *,
        config: Dict[str, object],
        target_search=None,
    ) -> None:
        self.config = OcclusionRecoveryConfig.from_config(config)
        self._target_search = target_search
        self.reset()

    def reset(self) -> None:
        self.state = "INITIAL_ACQUIRE"
        self.ever_target_locked = False
        self.target_lock_frames = 0
        self.reacquire_frames = 0
        self.last_target_bbox: Optional[Box] = None
        self.last_target_seen_at: Optional[float] = None
        self._last_target_center: Optional[Tuple[float, float]] = None
        self._last_target_center_at: Optional[float] = None
        self._velocity_x = 0.0
        self._velocity_y = 0.0
        self._target_history: Deque[TargetMotionSample] = deque(
            maxlen=self.config.exit_history_frames
        )
        self.last_loss_cause = "NONE"
        self.last_loss_reason = ""
        self._phase_started_at: Optional[float] = None
        self._initial_scan_started_at: Optional[float] = None
        self._initial_scan_active_seconds = 0.0
        self._initial_scan_last_update_at: Optional[float] = None
        self._initial_rotation_progress_degrees = 0.0
        self._initial_rotation_last_yaw: Optional[float] = None
        self._occluder_bbox: Optional[Box] = None
        self._occluder_overlap = 0.0
        self._occlusion_frames = 0
        self._bypass_direction = "right"
        self._lateral_pulses = 0
        self._local_scan_origin_yaw: Optional[float] = None
        self._local_scan_last_yaw: Optional[float] = None
        self._local_scan_progress_degrees = 0.0
        self._local_scan_started_at: Optional[float] = None

    def propose(
        self,
        ctx: ArbitrationContext,
        observation: Optional[ObstacleResult],
        now: float,
    ) -> Optional[FeatureProposal]:
        """Return an owned proposal, or ``None`` to resume normal follow/search."""
        if not self.config.enabled:
            return None

        if self._fresh_target(ctx.target_result):
            return self._target_visible(ctx, now)
        return self._target_missing(ctx, observation, now)

    def _target_visible(
        self, ctx: ArbitrationContext, now: float
    ) -> Optional[FeatureProposal]:
        was_recovering = self.state in self._RECOVERY_STATES
        self._remember_target(ctx.target_result, ctx.frame_width, ctx.frame_height, now)

        if not self.ever_target_locked:
            # Candidate verification is a hover phase, not part of the fallback
            # timed yaw scan when heading telemetry is unavailable.
            self._initial_scan_last_update_at = now
            self.target_lock_frames += 1
            if self.target_lock_frames < self.config.initial_lock_frames:
                self.state = "INITIAL_LOCK_VERIFY"
                return self._hover(
                    self.state,
                    f"target lock {self.target_lock_frames}/{self.config.initial_lock_frames}",
                )
            self.ever_target_locked = True
            self.state = "TRACKING"
            self._clear_occlusion_evidence()
            return None

        if was_recovering:
            self.state = "REACQUIRE_VERIFY"
            self.reacquire_frames += 1
            if self.reacquire_frames < self.config.reacquire_frames:
                return self._hover(
                    self.state,
                    f"target identity {self.reacquire_frames}/{self.config.reacquire_frames}",
                )
            self._reset_search_after_reacquire()
            self.state = "TRACKING"
            self.reacquire_frames = 0
            self._lateral_pulses = 0
            self._clear_occlusion_evidence()
            return None

        self.target_lock_frames = min(
            self.config.initial_lock_frames, self.target_lock_frames + 1
        )
        self.state = "TRACKING"
        return None

    def _target_missing(
        self,
        ctx: ArbitrationContext,
        observation: Optional[ObstacleResult],
        now: float,
    ) -> Optional[FeatureProposal]:
        self.target_lock_frames = 0
        self.reacquire_frames = 0

        if not self.ever_target_locked:
            self.state = "INITIAL_ACQUIRE"
            if self._initial_scan_started_at is None:
                self._initial_scan_started_at = now
                self._initial_scan_last_update_at = now
            acquire_elapsed = now - self._initial_scan_started_at
            if acquire_elapsed >= self.config.initial_acquire_timeout_seconds:
                return FeatureProposal(
                    RCCommand(),
                    state="INITIAL_ACQUIRE_TIMEOUT",
                    reason="no stable target acquired within bounded scan",
                    feature=self.feature_name,
                    requires_landing=True,
                    landing_kind="target_lost",
                )
            if observation is not None and observation.data_quality != "ok":
                return self._hover(
                    "OBSTACLE_SENSOR_HOLD",
                    f"obstacle observation unavailable: {observation.data_quality}",
                )
            self._advance_initial_scan_clock(now)
            telemetry_available = self._update_initial_rotation(ctx.yaw_deg)
            rotation_complete = (
                self._initial_rotation_progress_degrees
                >= self.config.initial_scan_degrees
                if telemetry_available
                else self._initial_scan_active_seconds
                >= self.config.initial_scan_fallback_seconds
            )
            if rotation_complete:
                return FeatureProposal(
                    RCCommand(),
                    state="INITIAL_SCAN_COMPLETE",
                    reason="full 360 target acquisition scan completed without lock",
                    feature=self.feature_name,
                    requires_landing=True,
                    landing_kind="target_lost",
                )
            return FeatureProposal(
                RCCommand(yaw=self.config.initial_scan_yaw_speed),
                state=self.state,
                reason=(
                    "full-turn target acquisition "
                    f"{self._initial_rotation_progress_degrees:.0f}/"
                    f"{self.config.initial_scan_degrees:.0f} deg"
                    if telemetry_available
                    else "full-turn target acquisition "
                    f"fallback {self._initial_scan_active_seconds:.1f}/"
                    f"{self.config.initial_scan_fallback_seconds:.1f} s"
                ),
                feature=self.feature_name,
            )

        if self.state == "TRACKING":
            assessment = self._assess_exit_fov(ctx)
            if assessment.classification == "EXIT_FOV":
                self.last_loss_cause = "EXIT_FOV"
                self.last_loss_reason = assessment.reason
                self._start_last_direction_search(ctx, now)
                self.state = "FALLBACK_SEARCH"
                self._clear_occlusion_evidence()
                return None
            if assessment.classification == "EXIT_POSSIBLE":
                self.last_loss_cause = "EXIT_POSSIBLE"
                self.last_loss_reason = assessment.reason
                self._enter("LOSS_UNCERTAIN_HOLD", now)
                self._clear_occlusion_evidence()
                return self._hover(
                    self.state,
                    f"loss_cause=EXIT_POSSIBLE; {assessment.reason}",
                )
            self.last_loss_cause = "PENDING"
            self.last_loss_reason = assessment.reason
            self._enter("OCCLUSION_CHECK", now)
            self._clear_occlusion_evidence()
        elif self.state == "REACQUIRE_VERIFY":
            if self._lateral_pulses >= self.config.max_lateral_pulses:
                self.state = "FALLBACK_SEARCH"
                return None
            next_state = (
                "OCCLUSION_SETTLE" if self._lateral_pulses else "OCCLUSION_CHECK"
            )
            self._enter(next_state, now)

        if self.state == "LOSS_UNCERTAIN_HOLD":
            if self._elapsed(now) < self.config.unknown_loss_hover_seconds:
                return self._hover(
                    self.state,
                    f"loss_cause={self.last_loss_cause}; "
                    f"waiting for bounded fallback search",
                )
            self.state = "FALLBACK_SEARCH"
            self._start_last_direction_search(ctx, now)
            return None

        if self.state == "OCCLUSION_CHECK":
            predicted = self.predicted_target_bbox(
                now, ctx.frame_width, ctx.frame_height
            )
            match = self._match_occluder(observation, predicted)
            if match is not None:
                bbox, overlap = match
                self._observe_occluder(bbox, overlap)
            else:
                self._clear_occlusion_evidence()

            if self._occlusion_frames >= self.config.occlusion_confirm_frames:
                direction = self._choose_bypass_direction(observation, predicted)
                if direction is not None:
                    self.last_loss_cause = "OCCLUSION"
                    self.last_loss_reason = (
                        "persistent obstacle overlaps the predicted target"
                    )
                    self._bypass_direction = direction
                    self._lateral_pulses = 1
                    self._enter("OCCLUSION_BYPASS", now)
                    return self._lateral_peek()
                self._enter("OCCLUSION_NO_SAFE_ROUTE", now)
                return self._hover(
                    self.state,
                    "target-linked occluder confirmed; no safe lateral sector",
                )

            hold_limit = min(
                self.config.occlusion_check_seconds,
                self.config.unknown_loss_hover_seconds,
            )
            if self._elapsed(now) < hold_limit:
                progress = (
                    f"occluder {self._occlusion_frames}/"
                    f"{self.config.occlusion_confirm_frames}"
                )
                return self._hover(self.state, progress)
            self.last_loss_cause = "UNKNOWN"
            self.last_loss_reason = "no high-confidence occluder before timeout"
            self.state = "FALLBACK_SEARCH"
            self._start_last_direction_search(ctx, now)
            return None

        if self.state == "OCCLUSION_NO_SAFE_ROUTE":
            predicted = self.predicted_target_bbox(
                now, ctx.frame_width, ctx.frame_height
            )
            direction = self._choose_bypass_direction(observation, predicted)
            if direction is not None:
                self._bypass_direction = direction
                self._lateral_pulses = 1
                self._enter("OCCLUSION_BYPASS", now)
                return self._lateral_peek()
            if self._elapsed(now) < self.config.no_safe_route_seconds:
                return self._hover(
                    self.state,
                    "waiting for a safe lateral sector; full search inhibited",
                )
            return FeatureProposal(
                RCCommand(),
                state="OCCLUSION_NO_SAFE_ROUTE_LANDING",
                reason="confirmed occlusion but neither lateral sector became safe",
                feature=self.feature_name,
                requires_landing=True,
                landing_kind="target_lost",
            )

        if self.state == "OCCLUSION_BYPASS":
            if self._elapsed(now) < self.config.lateral_pulse_seconds:
                if not self._bypass_path_still_safe(observation):
                    self._enter("OCCLUSION_SETTLE", now)
                    return self._hover(
                        self.state, "selected lateral sector is no longer clear"
                    )
                return self._lateral_peek()
            self._enter("OCCLUSION_SETTLE", now)
            return self._hover(self.state, "lateral pulse complete; ReID recheck")

        if self.state == "OCCLUSION_SETTLE":
            if self._elapsed(now) < self.config.settle_seconds:
                return self._hover(self.state, "hovering for a stable ReID frame")
            self._start_local_scan(ctx.yaw_deg, now)
            return self._local_scan_out(ctx.yaw_deg, now)

        if self.state == "OCCLUSION_LOCAL_SCAN_OUT":
            return self._local_scan_out(ctx.yaw_deg, now)

        if self.state == "OCCLUSION_LOCAL_SCAN_HOLD":
            if self._elapsed(now) < self.config.local_scan_hold_seconds:
                return self._hover(
                    self.state, "local scan edge; hovering for ReID"
                )
            self._enter("OCCLUSION_LOCAL_SCAN_RETURN", now)
            return self._local_scan_return(ctx.yaw_deg, observation, now)

        if self.state == "OCCLUSION_LOCAL_SCAN_RETURN":
            return self._local_scan_return(ctx.yaw_deg, observation, now)

        # FALLBACK_SEARCH: the existing bounded yaw/height search owns the tick.
        return None

    def predicted_target_bbox(
        self, now: float, frame_width: int, frame_height: int
    ) -> Optional[Box]:
        """Project the last trustworthy target box with a capped constant velocity."""
        if self.last_target_bbox is None or self.last_target_seen_at is None:
            return None
        age = max(0.0, now - self.last_target_seen_at)
        if age > self.config.occlusion_max_age_seconds:
            return None
        x, y, width, height = self.last_target_bbox
        dx = int(self._velocity_x * age)
        dy = int(self._velocity_y * age)
        max_x = max(0, frame_width - width)
        max_y = max(0, frame_height - height)
        return (
            max(0, min(max_x, x + dx)),
            max(0, min(max_y, y + dy)),
            width,
            height,
        )

    def _remember_target(
        self,
        result: Dict[str, object],
        frame_width: int,
        frame_height: int,
        now: float,
    ) -> None:
        bbox = self._valid_box(result.get("bbox"))
        if bbox is None:
            center = result.get("center")
            area = max(1.0, float(result.get("area") or 1.0))
            if not isinstance(center, (tuple, list)) or len(center) != 2:
                return
            side = max(8, int(area**0.5))
            bbox = (
                int(float(center[0]) - side / 2),
                int(float(center[1]) - side / 2),
                side,
                side,
            )
        x, y, width, height = bbox
        bbox = (
            max(0, min(max(0, frame_width - width), x)),
            max(0, min(max(0, frame_height - height), y)),
            min(width, max(1, frame_width)),
            min(height, max(1, frame_height)),
        )
        center = (bbox[0] + bbox[2] / 2.0, bbox[1] + bbox[3] / 2.0)
        if (
            self._target_history
            and now - self._target_history[-1].timestamp
            > self.config.occlusion_max_age_seconds
        ):
            self._target_history.clear()
        self._target_history.append(
            TargetMotionSample(
                timestamp=now,
                center_x=center[0],
                center_y=center[1],
                bbox=bbox,
            )
        )
        if self._last_target_center is not None and self._last_target_center_at is not None:
            dt = now - self._last_target_center_at
            if 0.01 <= dt <= self.config.occlusion_max_age_seconds:
                measured_x = (center[0] - self._last_target_center[0]) / dt
                measured_y = (center[1] - self._last_target_center[1]) / dt
                # EMA damps detector-box jitter while retaining recent direction.
                self._velocity_x = 0.65 * self._velocity_x + 0.35 * measured_x
                self._velocity_y = 0.65 * self._velocity_y + 0.35 * measured_y
            elif dt > self.config.occlusion_max_age_seconds:
                self._velocity_x = 0.0
                self._velocity_y = 0.0
        self.last_target_bbox = bbox
        self.last_target_seen_at = now
        self._last_target_center = center
        self._last_target_center_at = now

    def _assess_exit_fov(self, ctx: ArbitrationContext) -> ExitAssessment:
        """Classify a loss before any target-linked lateral bypass is authorized.

        A target need not be running quickly in world coordinates.  Being near a
        horizontal image edge is enough to inhibit lateral recovery; consistent
        outward image motion, an increasing tracking error, or saturated yaw can
        promote that conservative hold to an immediate last-direction search.
        """
        if not self._target_history or ctx.frame_width <= 0:
            return ExitAssessment("NONE", "insufficient target trajectory")

        latest = self._target_history[-1]
        oldest = self._target_history[0]
        frame_width = float(ctx.frame_width)
        margin = frame_width * self.config.exit_edge_margin_ratio
        left_near = latest.bbox[0] <= margin
        right_near = latest.bbox[0] + latest.bbox[2] >= frame_width - margin
        horizontal_edge_near = left_near or right_near

        direction = -1 if latest.center_x < frame_width / 2.0 else 1
        displacement = latest.center_x - oldest.center_x
        outward_displacement = displacement * direction
        minimum_displacement = max(
            2.0, frame_width * self.config.exit_min_displacement_ratio
        )
        outward_trend = outward_displacement >= minimum_displacement

        elapsed = latest.timestamp - oldest.timestamp
        velocity = displacement / elapsed if elapsed >= 0.05 else 0.0
        outward_velocity = velocity * direction
        fast_outward = (
            outward_velocity / frame_width
            >= self.config.exit_min_velocity_ratio_per_second
        )
        distance_to_edge = (
            latest.center_x if direction < 0 else frame_width - latest.center_x
        )
        time_to_edge = (
            distance_to_edge / outward_velocity
            if outward_velocity > 1e-6
            else float("inf")
        )
        edge_arrival_soon = time_to_edge <= self.config.exit_time_to_edge_seconds

        old_error = abs(oldest.center_x - frame_width / 2.0)
        new_error = abs(latest.center_x - frame_width / 2.0)
        error_growing = new_error - old_error >= minimum_displacement
        last_yaw = self._last_yaw_command(ctx)
        yaw_threshold = max(
            1.0,
            self.config.maximum_yaw_speed
            * self.config.exit_yaw_saturation_ratio,
        )
        yaw_saturated_outward = (
            abs(last_yaw) >= yaw_threshold and last_yaw * direction > 0
        )

        confirmed = (
            (horizontal_edge_near and outward_trend)
            or (yaw_saturated_outward and error_growing)
            or (outward_trend and fast_outward and edge_arrival_soon)
        )
        evidence = (
            f"edge={horizontal_edge_near}, outward={outward_trend}, "
            f"yaw_saturated={yaw_saturated_outward}, "
            f"error_growing={error_growing}, t_edge={time_to_edge:.2f}s"
        )
        if confirmed:
            return ExitAssessment("EXIT_FOV", evidence)
        if horizontal_edge_near or outward_trend or yaw_saturated_outward:
            return ExitAssessment("EXIT_POSSIBLE", evidence)
        return ExitAssessment("NONE", evidence)

    @staticmethod
    def _last_yaw_command(ctx: ArbitrationContext) -> int:
        """Read the previous emitted yaw command from optional kernel context."""
        command = ctx.extra.get("last_command")
        if isinstance(command, RCCommand):
            return int(command.yaw)
        if isinstance(command, dict):
            try:
                return int(command.get("yaw", 0))
            except (TypeError, ValueError):
                return 0
        if isinstance(command, (tuple, list)) and len(command) == 4:
            try:
                return int(command[3])
            except (TypeError, ValueError):
                return 0
        return 0

    def _match_occluder(
        self,
        observation: Optional[ObstacleResult],
        predicted: Optional[Box],
    ) -> Optional[Tuple[Box, float]]:
        if (
            observation is None
            or not observation.found
            or observation.state not in {"CAUTION", "BLOCKED"}
            or predicted is None
        ):
            return None
        expanded = self._expand_box(predicted, self.config.prediction_margin_ratio)
        candidates = [
            (candidate.bbox, candidate.area_ratio)
            for candidate in observation.candidates
        ]
        candidate_boxes = [bbox for bbox, _area_ratio in candidates]
        if observation.bbox is not None and observation.bbox not in candidate_boxes:
            candidates.append((observation.bbox, observation.area_ratio))
        target_area = max(1, predicted[2] * predicted[3])
        best: Optional[Tuple[Box, float]] = None
        for raw_bbox, area_ratio in candidates:
            bbox = self._valid_box(raw_bbox)
            if bbox is None:
                continue
            # BLOCKED remains valid under the global obstacle threshold. CAUTION
            # is accepted only as target-linked occlusion evidence when its own
            # contour is materially large; this avoids promoting every edge/noise
            # candidate while fixing real boards that never reach area_ratio=0.18.
            if (
                observation.state == "CAUTION"
                and area_ratio < self.config.occluder_min_area_ratio
            ):
                continue
            overlap = self._intersection_area(expanded, bbox) / target_area
            overlap = min(1.0, overlap)
            if best is None or overlap > best[1]:
                best = (bbox, overlap)
        if best is None or best[1] < self.config.occlusion_overlap_ratio:
            return None
        return best

    def _observe_occluder(self, bbox: Box, overlap: float) -> None:
        persistent = (
            self._occluder_bbox is not None
            and self._box_iou(self._occluder_bbox, bbox)
            >= self.config.occluder_iou_threshold
        )
        self._occlusion_frames = self._occlusion_frames + 1 if persistent else 1
        self._occluder_bbox = bbox
        self._occluder_overlap = overlap

    def _choose_bypass_direction(
        self,
        observation: Optional[ObstacleResult],
        predicted: Optional[Box],
    ) -> Optional[str]:
        if observation is None or self._occluder_bbox is None:
            return None
        left_score, right_score, has_space_map = self._free_space_scores(
            observation.free_space
        )
        if has_space_map and max(left_score, right_score) < self.config.min_free_space_score:
            return None

        preferred = self._bypass_direction
        if predicted is not None:
            target_x = predicted[0] + predicted[2] / 2.0
            obstacle_x = self._occluder_bbox[0] + self._occluder_bbox[2] / 2.0
            preferred = "left" if target_x < obstacle_x else "right"
        elif observation.side == "left":
            preferred = "right"
        elif observation.side == "right":
            preferred = "left"

        # Free-space evidence dominates; predicted target side only breaks close ties.
        left_total = 0.75 * left_score + (0.25 if preferred == "left" else 0.0)
        right_total = 0.75 * right_score + (0.25 if preferred == "right" else 0.0)
        return "right" if right_total > left_total else "left"

    @staticmethod
    def _free_space_scores(free_space: Dict[str, float]) -> Tuple[float, float, bool]:
        if not free_space:
            return 1.0, 1.0, False
        left = [
            float(value)
            for key, value in free_space.items()
            if key in {"far_left", "left"}
        ]
        right = [
            float(value)
            for key, value in free_space.items()
            if key in {"right", "far_right"}
        ]
        if not left and not right:
            indexed = sorted(
                (
                    int("".join(ch for ch in key if ch.isdigit())),
                    float(value),
                )
                for key, value in free_space.items()
                if any(ch.isdigit() for ch in key)
            )
            count = len(indexed)
            left = [value for index, value in indexed if index < count // 2]
            right = [
                value for index, value in indexed if index >= count - count // 2
            ]
        return (
            sum(left) / len(left) if left else 0.0,
            sum(right) / len(right) if right else 0.0,
            bool(left or right),
        )

    def _lateral_peek(self) -> FeatureProposal:
        sign = 1 if self._bypass_direction == "right" else -1
        return FeatureProposal(
            RCCommand(left_right=sign * self.config.lateral_speed),
            state="OCCLUSION_BYPASS",
            reason=(
                f"occluder bbox={self._occluder_bbox} "
                f"overlap={self._occluder_overlap:.2f}; "
                f"lateral {self._bypass_direction} pulse "
                f"{self._lateral_pulses}/{self.config.max_lateral_pulses}"
            ),
            feature=self.feature_name,
        )

    def _start_local_scan(self, yaw_deg: Optional[int], now: float) -> None:
        """Begin a small yaw scan opposite to the completed lateral peek."""
        self._local_scan_origin_yaw = None if yaw_deg is None else float(yaw_deg)
        self._local_scan_last_yaw = self._local_scan_origin_yaw
        self._local_scan_progress_degrees = 0.0
        self._local_scan_started_at = now
        self._enter("OCCLUSION_LOCAL_SCAN_OUT", now)

    def _local_scan_out(
        self, yaw_deg: Optional[int], now: float
    ) -> FeatureProposal:
        """Yaw toward the side where the target should appear after translation."""
        direction = -1 if self._bypass_direction == "right" else 1
        telemetry_available = self._update_local_rotation(yaw_deg, direction)
        elapsed = now - (
            self._local_scan_started_at
            if self._local_scan_started_at is not None
            else now
        )
        complete = (
            self._local_scan_progress_degrees >= self.config.local_scan_degrees
            if telemetry_available
            else elapsed >= self.config.local_scan_fallback_seconds
        )
        if complete:
            self._enter("OCCLUSION_LOCAL_SCAN_HOLD", now)
            return self._hover(
                self.state,
                f"local scan reached {self.config.local_scan_degrees:.0f} deg",
            )
        return FeatureProposal(
            RCCommand(yaw=direction * self.config.local_scan_yaw_speed),
            state="OCCLUSION_LOCAL_SCAN_OUT",
            reason=(
                f"local scan opposite lateral motion "
                f"{self._local_scan_progress_degrees:.0f}/"
                f"{self.config.local_scan_degrees:.0f} deg"
                if telemetry_available
                else "local scan opposite lateral motion "
                f"fallback {elapsed:.1f}/"
                f"{self.config.local_scan_fallback_seconds:.1f} s"
            ),
            feature=self.feature_name,
        )

    def _local_scan_return(
        self,
        yaw_deg: Optional[int],
        observation: Optional[ObstacleResult],
        now: float,
    ) -> Optional[FeatureProposal]:
        """Return to the pre-scan heading before another lateral pulse/search."""
        out_direction = -1 if self._bypass_direction == "right" else 1
        return_direction = -out_direction
        elapsed = self._elapsed(now)
        telemetry_available = (
            yaw_deg is not None and self._local_scan_origin_yaw is not None
        )
        if telemetry_available:
            error = abs(
                self._signed_yaw_delta(
                    float(yaw_deg), self._local_scan_origin_yaw
                )
            )
            complete = error <= self.config.local_scan_return_tolerance_degrees
            progress = f"heading error={error:.1f} deg"
        else:
            complete = elapsed >= self.config.local_scan_fallback_seconds
            progress = (
                f"fallback {elapsed:.1f}/"
                f"{self.config.local_scan_fallback_seconds:.1f} s"
            )
        if complete:
            return self._after_local_scan(observation, now)
        return FeatureProposal(
            RCCommand(yaw=return_direction * self.config.local_scan_yaw_speed),
            state="OCCLUSION_LOCAL_SCAN_RETURN",
            reason=f"returning to pre-scan heading; {progress}",
            feature=self.feature_name,
        )

    def _after_local_scan(
        self, observation: Optional[ObstacleResult], now: float
    ) -> Optional[FeatureProposal]:
        """Start the next bounded peek, or release to the full fallback search."""
        if self._lateral_pulses < self.config.max_lateral_pulses:
            if not self._bypass_path_still_safe(observation):
                self.state = "FALLBACK_SEARCH"
                return self._hover(
                    "OCCLUSION_ROUTE_BLOCKED",
                    "lateral sector no longer safe after local scan",
                )
            self._lateral_pulses += 1
            self._enter("OCCLUSION_BYPASS", now)
            return self._lateral_peek()
        self.state = "FALLBACK_SEARCH"
        return None

    def _bypass_path_still_safe(
        self, observation: Optional[ObstacleResult]
    ) -> bool:
        if observation is None or observation.data_quality != "ok":
            return False
        left_score, right_score, has_space_map = self._free_space_scores(
            observation.free_space
        )
        if not has_space_map:
            return True
        selected_score = (
            right_score if self._bypass_direction == "right" else left_score
        )
        return selected_score >= self.config.min_free_space_score

    def _advance_initial_scan_clock(self, now: float) -> None:
        """Accumulate only ticks that actively belong to initial acquisition."""
        if self._initial_scan_last_update_at is not None:
            delta = now - self._initial_scan_last_update_at
            if 0.0 <= delta <= 1.0:
                self._initial_scan_active_seconds += delta
        self._initial_scan_last_update_at = now

    def _update_initial_rotation(self, yaw_deg: Optional[int]) -> bool:
        """Accumulate one commanded full turn through the -180/180 yaw wrap."""
        if yaw_deg is None:
            self._initial_rotation_last_yaw = None
            return False
        current = float(yaw_deg)
        if self._initial_rotation_last_yaw is None:
            self._initial_rotation_last_yaw = current
            return True
        delta = self._signed_yaw_delta(current, self._initial_rotation_last_yaw)
        self._initial_rotation_last_yaw = current
        if delta > 0:
            self._initial_rotation_progress_degrees += delta
        return True

    def _update_local_rotation(
        self, yaw_deg: Optional[int], direction: int
    ) -> bool:
        """Accumulate local commanded yaw while rejecting opposite drift."""
        if yaw_deg is None:
            self._local_scan_last_yaw = None
            return False
        current = float(yaw_deg)
        if self._local_scan_last_yaw is None:
            self._local_scan_last_yaw = current
            return True
        delta = self._signed_yaw_delta(current, self._local_scan_last_yaw)
        self._local_scan_last_yaw = current
        commanded_delta = delta * direction
        if commanded_delta > 0:
            self._local_scan_progress_degrees += commanded_delta
        return True

    @staticmethod
    def _signed_yaw_delta(current: float, previous: float) -> float:
        """Return the shortest signed heading change across telemetry wrap."""
        return ((current - previous + 180.0) % 360.0) - 180.0

    def _hover(self, state: str, reason: str) -> FeatureProposal:
        return FeatureProposal(
            RCCommand(), state=state, reason=reason, feature=self.feature_name
        )

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self._phase_started_at = now

    def _elapsed(self, now: float) -> float:
        return now - (self._phase_started_at if self._phase_started_at is not None else now)

    def _clear_occlusion_evidence(self) -> None:
        self._occluder_bbox = None
        self._occluder_overlap = 0.0
        self._occlusion_frames = 0

    def _reset_search_after_reacquire(self) -> None:
        reset = getattr(self._target_search, "reset", None)
        if callable(reset):
            reset()

    def _start_last_direction_search(
        self, ctx: ArbitrationContext, now: float
    ) -> None:
        """Prime target search after this feature has already performed a hold."""
        start = getattr(self._target_search, "start_last_direction_search", None)
        if not callable(start):
            return
        height_cm = int(ctx.height_cm) if ctx.height_cm is not None else None
        start(now, height_cm)

    @staticmethod
    def _fresh_target(result: Dict[str, object]) -> bool:
        return (
            bool(result.get("found"))
            and not bool(result.get("is_predicted"))
            and not bool(result.get("ambiguous"))
        )

    @staticmethod
    def _valid_box(value: object) -> Optional[Box]:
        if not isinstance(value, (tuple, list)) or len(value) != 4:
            return None
        try:
            x, y, width, height = (int(float(item)) for item in value)
        except (TypeError, ValueError):
            return None
        if width <= 0 or height <= 0:
            return None
        return x, y, width, height

    @staticmethod
    def _expand_box(box: Box, ratio: float) -> Box:
        x, y, width, height = box
        pad_x = int(width * ratio)
        pad_y = int(height * ratio)
        return x - pad_x, y - pad_y, width + 2 * pad_x, height + 2 * pad_y

    @staticmethod
    def _intersection_area(first: Box, second: Box) -> int:
        x1 = max(first[0], second[0])
        y1 = max(first[1], second[1])
        x2 = min(first[0] + first[2], second[0] + second[2])
        y2 = min(first[1] + first[3], second[1] + second[3])
        return max(0, x2 - x1) * max(0, y2 - y1)

    @classmethod
    def _box_iou(cls, first: Box, second: Box) -> float:
        intersection = cls._intersection_area(first, second)
        union = first[2] * first[3] + second[2] * second[3] - intersection
        return intersection / max(1, union)


__all__ = ["OcclusionRecoveryConfig", "OcclusionRecoveryFeature"]
