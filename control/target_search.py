"""Bounded and predictable target-loss recovery for ReID follow sessions."""

from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from control.follow_control import RCCommand
from vision.reid_enrollment import TargetLockTracker


@dataclass(frozen=True)
class SearchDecision:
    """One search-state output consumed by FollowSession."""

    command: RCCommand
    action: str
    state: str
    reason: str = ""


class TargetSearchController:
    """Run one simple search loop without blind forward or lateral flight.

    The fixed flow is: hold -> optional close backoff -> last horizontal
    direction -> current/upper/lower layer scans -> return to base -> land.
    """

    _LAYER_OFFSETS = (0, 1, -1)

    def __init__(self, config: Dict[str, object], min_height_cm: int, max_height_cm: int) -> None:
        search = config.get("target_search", {})
        cfg = search if isinstance(search, dict) else {}
        self.enabled = bool(cfg.get("enabled", True))
        self.hold_seconds = self._positive_float(cfg.get("hold_seconds"), 1.0)
        self.yaw_speed = self._positive_int(cfg.get("yaw_speed"), 30)
        self.last_direction_yaw_speed = self._positive_int(
            cfg.get("last_direction_yaw_speed"), 25
        )
        self.vertical_speed = self._positive_int(cfg.get("vertical_speed"), 20)
        self.height_step_cm = self._positive_int(cfg.get("height_step_cm"), 20)
        self.min_height_cm = max(int(min_height_cm), int(cfg.get("min_height_cm", 80)))
        self.max_height_cm = min(int(max_height_cm), int(cfg.get("max_height_cm", 200)))
        self.reacquire_frames = self._positive_int(cfg.get("reacquire_frames"), 5)
        self.last_direction_seconds = self._positive_float(
            cfg.get("last_direction_seconds"), 2.0
        )
        self.full_rotation_degrees = self._positive_float(
            cfg.get("full_rotation_degrees"), 360.0
        )
        self.full_rotation_fallback_seconds = self._positive_float(
            cfg.get("full_rotation_fallback_seconds"),
            self.full_rotation_degrees / max(1, self.yaw_speed),
        )
        self.close_area_ratio = self._positive_float(
            cfg.get("close_area_ratio"),
            float(config.get("target_area_ratio_max", 0.30)),
        )
        self.close_very_area_ratio = max(
            self.close_area_ratio,
            self._positive_float(cfg.get("close_very_area_ratio"), 0.40),
        )
        self.close_recovery_enabled = bool(cfg.get("close_recovery_enabled", True))
        self.close_backward_speed = self._positive_int(
            cfg.get("close_backward_speed"), 35
        )
        self.close_pulse_seconds = self._positive_float(
            cfg.get("close_pulse_seconds"), 1.5
        )
        self.close_pause_seconds = self._positive_float(
            cfg.get("close_pause_seconds"), 0.50
        )
        self.close_max_attempts = self._positive_int(cfg.get("close_max_attempts"), 2)
        self.edge_margin_ratio = min(
            0.25, self._positive_float(cfg.get("edge_margin_ratio"), 0.03)
        )

        self.state = "IDLE"
        self.phase_started_at: Optional[float] = None
        self.verification_started_at: Optional[float] = None
        self.search_height_cm: Optional[int] = None
        self.layer_index = 0
        self.has_observed_target = False
        self.last_horizontal_direction = 1
        self.last_area_ratio = 0.0
        self.last_bbox: Optional[Tuple[int, int, int, int]] = None
        self.last_frame_size: Tuple[int, int] = (0, 0)
        self.last_command = RCCommand()
        self.area_history: deque[float] = deque(maxlen=4)
        self.close_attempts = 0
        self.rotation_progress_degrees = 0.0
        self.rotation_last_yaw: Optional[float] = None
        self.horizontal_edge_exit_active = False
        self._reacquire_tracker = TargetLockTracker(self.reacquire_frames)

    @property
    def searching(self) -> bool:
        return self.state != "IDLE"

    def close_recovery_has_priority(self) -> bool:
        """Whether the narrowly scoped too-close recovery must preempt avoidance."""
        if not self.enabled or not self.close_recovery_enabled or not self.has_observed_target:
            return False
        if self.state in {"CLOSE_BACKOFF", "CLOSE_BACKOFF_PAUSE"}:
            return True
        if self.state not in {"IDLE", "LOST_HOLD"}:
            return False
        return self.close_attempts < self.close_max_attempts and self._looks_too_close()

    def visible_close_recovery_has_priority(
        self, result: Optional[Dict[str, object]] = None, frame_width: int = 0, frame_height: int = 0
    ) -> bool:
        """Whether a visible, oversized target must be backed away from."""
        if not self.enabled or not self.close_recovery_enabled:
            return False
        if result is None:
            return self.has_observed_target and self.last_area_ratio >= self.close_area_ratio
        if not self._fresh_target(result):
            return False
        area_ratio = float(result.get("area_ratio") or 0.0)
        if area_ratio <= 0 and frame_width > 0 and frame_height > 0:
            area_ratio = float(result.get("area") or 0.0) / (frame_width * frame_height)
        return area_ratio >= self.close_area_ratio

    def visible_close_recovery_command(self) -> RCCommand:
        """Return the bounded reverse command for a visible close target."""
        return RCCommand(forward_backward=-self.close_backward_speed)

    def horizontal_edge_exit_has_priority(self) -> bool:
        """Keep a left/right frame exit in search, ahead of all ToF avoidance."""
        if not self.enabled or not self.has_observed_target:
            return False
        if self.searching:
            return self.horizontal_edge_exit_active
        return self._last_bbox_touches_horizontal_edge()

    def reset(self) -> None:
        self.state = "IDLE"
        self.phase_started_at = None
        self.verification_started_at = None
        self.search_height_cm = None
        self.layer_index = 0
        self.has_observed_target = False
        self.horizontal_edge_exit_active = False
        self.close_attempts = 0
        self._reset_rotation_tracking()
        self._reacquire_tracker.reset()

    def observe_target(
        self,
        result: Dict[str, object],
        frame_width: int,
        frame_height: int,
        command: RCCommand,
    ) -> None:
        """Remember the last trustworthy target pose before it disappears."""
        if not self._fresh_target(result):
            return
        self.has_observed_target = True
        center = result.get("center")
        if center is not None:
            x, _y = center  # type: ignore[misc]
            horizontal_error = int(x) - frame_width // 2
            if horizontal_error:
                self.last_horizontal_direction = 1 if horizontal_error > 0 else -1
        frame_area = max(1, frame_width * frame_height)
        area_ratio = float(result.get("area_ratio") or 0.0)
        if area_ratio <= 0:
            area_ratio = float(result.get("area") or 0.0) / frame_area
        self.last_area_ratio = area_ratio
        self.area_history.append(area_ratio)
        bbox = result.get("bbox")
        self.last_bbox = tuple(int(value) for value in bbox) if bbox is not None else None
        self.last_frame_size = (frame_width, frame_height)
        self.last_command = command

    def update(
        self,
        result: Dict[str, object],
        frame_width: int,
        frame_height: int,
        height_cm: Optional[int],
        now: float,
        yaw_deg: Optional[int] = None,
    ) -> Optional[SearchDecision]:
        """Return None for following, otherwise the current bounded search action."""
        del frame_width, frame_height  # Kept in the shared controller interface.
        if not self.enabled:
            return None

        if not self.searching:
            if self._fresh_target(result):
                return None
            self._start_search(now, height_cm)

        if self._fresh_target(result):
            if self.verification_started_at is None:
                self.verification_started_at = now
            if self._reacquire_tracker.observe(result):
                self.reset()
                return SearchDecision(RCCommand(), "reacquired", "TARGET_REACQUIRED")
            return SearchDecision(
                RCCommand(),
                "search",
                "REACQUIRE_VERIFY",
                f"ReID {self._reacquire_tracker.progress}",
            )

        self._resume_phase_timer_after_verification(now)
        self._reacquire_tracker.reset()
        elapsed = now - (self.phase_started_at if self.phase_started_at is not None else now)

        if self.state == "LOST_HOLD":
            if elapsed < self.hold_seconds:
                return self._hover("brief hold")
            if (
                not self.horizontal_edge_exit_active
                and self.close_recovery_enabled
                and self._looks_too_close()
            ):
                next_state = "CLOSE_BACKOFF"
            elif self.has_observed_target:
                next_state = "SEARCH_LAST_DIRECTION"
            else:
                self.layer_index = 0
                next_state = "MOVE_TO_LAYER"
            self._enter(next_state, now)
            elapsed = 0.0

        if self.state == "CLOSE_BACKOFF":
            if elapsed < self.close_pulse_seconds:
                return SearchDecision(
                    RCCommand(forward_backward=-self.close_backward_speed),
                    "search",
                    self.state,
                    "bounded close-target recovery",
                )
            self._enter("CLOSE_BACKOFF_PAUSE", now)
            return self._hover("recheck target")

        if self.state == "CLOSE_BACKOFF_PAUSE":
            if elapsed < self.close_pause_seconds:
                return self._hover("recheck target")
            self.close_attempts += 1
            next_state = (
                "CLOSE_BACKOFF"
                if self.close_attempts < self.close_max_attempts
                else "SEARCH_LAST_DIRECTION"
            )
            self._enter(next_state, now)
            return self._hover("close recovery complete")

        if self.state == "SEARCH_LAST_DIRECTION":
            if elapsed < self.last_direction_seconds:
                return SearchDecision(
                    RCCommand(
                        yaw=self.last_direction_yaw_speed
                        * self.last_horizontal_direction
                    ),
                    "search",
                    self.state,
                    "last horizontal direction",
                )
            self.layer_index = 0
            self._enter("MOVE_TO_LAYER", now)
            return self._hover("start fixed layer loop")

        if self.state == "MOVE_TO_LAYER":
            return self._move_to_layer(height_cm, now)

        if self.state == "LAYER_SCAN_FULL":
            direction = self._layer_scan_direction()
            telemetry_available = self._update_rotation_progress(yaw_deg, direction)
            rotation_complete = (
                self.rotation_progress_degrees >= self.full_rotation_degrees
                if telemetry_available
                else elapsed >= self.full_rotation_fallback_seconds
            )
            if not rotation_complete:
                progress = (
                    f"{self.rotation_progress_degrees:.0f}/{self.full_rotation_degrees:.0f} deg"
                    if telemetry_available
                    else f"fallback {elapsed:.1f}/{self.full_rotation_fallback_seconds:.1f} s"
                )
                return self._yaw_decision(direction, f"full rotation {progress}")
            if self.layer_index + 1 < len(self._LAYER_OFFSETS):
                self.layer_index += 1
                self._enter("MOVE_TO_LAYER", now)
                return self._hover("next search layer")
            self._enter("RETURN_TO_BASE", now)
            return self._hover("full search round complete; return to base")

        if self.state == "RETURN_TO_BASE":
            return self._return_to_base(height_cm)

        return self._hover("search hold")

    def _move_to_layer(self, height_cm: Optional[int], now: float) -> SearchDecision:
        target = self._layer_target_cm()
        if height_cm is None:
            return self._hover("waiting for TOF")
        error = target - height_cm
        if abs(error) <= 5:
            self._enter("LAYER_SCAN_FULL", now)
            self._reset_rotation_tracking()
            return self._hover(f"layer {self.layer_index + 1}/3 ready")
        direction = 1 if error > 0 else -1
        return SearchDecision(
            RCCommand(up_down=self.vertical_speed * direction),
            "search",
            self.state,
            f"move to layer {self.layer_index + 1}/3: {target} cm",
        )

    def _return_to_base(self, height_cm: Optional[int]) -> SearchDecision:
        target = self.search_height_cm if self.search_height_cm is not None else 150
        if height_cm is None:
            return self._hover("waiting for TOF before landing")
        error = target - height_cm
        if abs(error) <= 5:
            self.state = "SEARCH_COMPLETE_LANDING"
            return SearchDecision(
                RCCommand(),
                "land",
                self.state,
                "full search round complete",
            )
        direction = 1 if error > 0 else -1
        return SearchDecision(
            RCCommand(up_down=self.vertical_speed * direction),
            "search",
            self.state,
            f"return to search base height: {target} cm",
        )

    def _layer_target_cm(self) -> int:
        base = self.search_height_cm if self.search_height_cm is not None else 150
        offset = self._LAYER_OFFSETS[self.layer_index] * self.height_step_cm
        return max(self.min_height_cm, min(self.max_height_cm, base + offset))

    def _layer_scan_direction(self) -> int:
        """Alternate full-turn direction between adjacent height layers."""
        base_direction = self.last_horizontal_direction if self.has_observed_target else 1
        return base_direction if self.layer_index % 2 == 0 else -base_direction

    def _update_rotation_progress(self, yaw_deg: Optional[int], direction: int) -> bool:
        """Accumulate commanded yaw through the -180/180 telemetry wrap."""
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

    def _start_search(self, now: float, height_cm: Optional[int]) -> None:
        base_height = 150 if height_cm is None else int(height_cm)
        self.search_height_cm = max(
            self.min_height_cm, min(self.max_height_cm, base_height)
        )
        self.phase_started_at = now
        self.verification_started_at = None
        self.layer_index = 0
        self.close_attempts = 0
        self.horizontal_edge_exit_active = self._last_bbox_touches_horizontal_edge()
        self._reset_rotation_tracking()
        self._reacquire_tracker.reset()
        self.state = "LOST_HOLD"

    def _resume_phase_timer_after_verification(self, now: float) -> None:
        if self.verification_started_at is None:
            return
        if self.phase_started_at is not None:
            self.phase_started_at += now - self.verification_started_at
        self.verification_started_at = None

    def _looks_too_close(self) -> bool:
        growing = (
            len(self.area_history) >= 2
            and self.area_history[-1] > self.area_history[0] * 1.08
        )
        clipped = self._last_bbox_touches_edge()
        distance_recovery_active = self.last_command.forward_backward < 0
        return (
            self.last_area_ratio >= self.close_very_area_ratio
            or (
                self.last_area_ratio >= self.close_area_ratio
                and (growing or clipped or distance_recovery_active)
            )
        )

    def _last_bbox_touches_edge(self) -> bool:
        if self.last_bbox is None:
            return False
        width, height = self.last_frame_size
        if width <= 0 or height <= 0:
            return False
        x, y, box_width, box_height = self.last_bbox
        margin_x = width * self.edge_margin_ratio
        margin_y = height * self.edge_margin_ratio
        return (
            x <= margin_x
            or y <= margin_y
            or x + box_width >= width - margin_x
            or y + box_height >= height - margin_y
        )

    def _last_bbox_touches_horizontal_edge(self) -> bool:
        """Return true only for exits through the left or right image border."""
        if self.last_bbox is None:
            return False
        width, _height = self.last_frame_size
        if width <= 0:
            return False
        x, _y, box_width, _box_height = self.last_bbox
        margin_x = width * self.edge_margin_ratio
        touches_left = x <= margin_x
        touches_right = x + box_width >= width - margin_x
        center_x = x + box_width / 2.0
        # A lateral exit must touch exactly one side and have its box centre in
        # that side's outer quarter. A huge close target can touch one/both
        # borders while its centre remains near the image centre; that stays in
        # the existing too-close recovery instead of being mistaken for exit.
        return (
            touches_left and not touches_right and center_x <= width * 0.25
        ) or (
            touches_right and not touches_left and center_x >= width * 0.75
        )

    def _yaw_decision(self, direction: int, reason: str) -> SearchDecision:
        return SearchDecision(
            RCCommand(yaw=self.yaw_speed * direction),
            "search",
            self.state,
            reason,
        )

    def _hover(self, reason: str) -> SearchDecision:
        return SearchDecision(RCCommand(), "search", self.state, reason)

    def _enter(self, state: str, now: float) -> None:
        self.state = state
        self.phase_started_at = now

    @staticmethod
    def _fresh_target(result: Dict[str, object]) -> bool:
        return (
            bool(result.get("found"))
            and not bool(result.get("is_predicted"))
            and not bool(result.get("ambiguous"))
        )

    @staticmethod
    def _positive_float(value: object, default: float) -> float:
        try:
            parsed = float(value)
        except (TypeError, ValueError):
            parsed = default
        return max(0.01, parsed)

    @staticmethod
    def _positive_int(value: object, default: int) -> int:
        try:
            parsed = int(value)
        except (TypeError, ValueError):
            parsed = default
        return max(1, parsed)
