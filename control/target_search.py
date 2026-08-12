"""Bounded target-loss recovery for appearance-based follow sessions."""

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
    """Search for a lost ReID target without blind forward or lateral flight."""

    def __init__(self, config: Dict[str, object], min_height_cm: int, max_height_cm: int) -> None:
        search = config.get("target_search", {})
        cfg = search if isinstance(search, dict) else {}
        self.enabled = bool(cfg.get("enabled", True))
        self.hold_seconds = self._positive_float(cfg.get("hold_seconds"), 1.0)
        self.total_timeout_seconds = self._positive_float(cfg.get("total_timeout_seconds"), 30.0)
        self.yaw_speed = self._positive_int(cfg.get("yaw_speed"), 12)
        self.vertical_speed = self._positive_int(cfg.get("vertical_speed"), 12)
        self.height_step_cm = self._positive_int(cfg.get("height_step_cm"), 20)
        self.min_height_cm = max(int(min_height_cm), int(cfg.get("min_height_cm", 80)))
        self.max_height_cm = min(int(max_height_cm), int(cfg.get("max_height_cm", 200)))
        self.reacquire_frames = self._positive_int(cfg.get("reacquire_frames"), 5)
        self.last_direction_seconds = self._positive_float(cfg.get("last_direction_seconds"), 2.0)
        self.sweep_short_seconds = self._positive_float(cfg.get("sweep_short_seconds"), 1.5)
        self.sweep_long_seconds = self._positive_float(cfg.get("sweep_long_seconds"), 3.0)
        self.close_area_ratio = self._positive_float(
            cfg.get("close_area_ratio"),
            float(config.get("target_area_ratio_max", 0.30)),
        )
        self.close_very_area_ratio = max(
            self.close_area_ratio,
            self._positive_float(cfg.get("close_very_area_ratio"), 0.40),
        )
        self.close_recovery_enabled = bool(cfg.get("close_recovery_enabled", True))
        self.close_backward_speed = self._positive_int(cfg.get("close_backward_speed"), 10)
        self.close_pulse_seconds = self._positive_float(cfg.get("close_pulse_seconds"), 0.35)
        self.close_pause_seconds = self._positive_float(cfg.get("close_pause_seconds"), 0.50)
        self.close_max_attempts = self._positive_int(cfg.get("close_max_attempts"), 2)
        self.edge_margin_ratio = min(
            0.25, self._positive_float(cfg.get("edge_margin_ratio"), 0.03)
        )

        self.state = "IDLE"
        self.search_started_at: Optional[float] = None
        self.phase_started_at: Optional[float] = None
        self.search_height_cm: Optional[int] = None
        self.last_horizontal_direction = 1
        self.last_vertical_direction = 0
        self.last_search_axis = "horizontal"
        self.last_area_ratio = 0.0
        self.last_bbox: Optional[Tuple[int, int, int, int]] = None
        self.last_frame_size: Tuple[int, int] = (0, 0)
        self.last_command = RCCommand()
        self.area_history: deque[float] = deque(maxlen=4)
        self.close_attempts = 0
        self._reacquire_tracker = TargetLockTracker(self.reacquire_frames)

    @property
    def searching(self) -> bool:
        return self.state != "IDLE"

    def reset(self) -> None:
        self.state = "IDLE"
        self.search_started_at = None
        self.phase_started_at = None
        self.search_height_cm = None
        self.close_attempts = 0
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
        center = result.get("center")
        if center is not None:
            x, y = center  # type: ignore[misc]
            horizontal_error = int(x) - frame_width // 2
            vertical_error = int(y) - frame_height // 2
            if horizontal_error:
                self.last_horizontal_direction = 1 if horizontal_error > 0 else -1
            if vertical_error:
                # Target above the image suggests searching upward first.
                self.last_vertical_direction = 1 if vertical_error < 0 else -1
            horizontal_ratio = abs(horizontal_error) / max(1, frame_width)
            vertical_ratio = abs(vertical_error) / max(1, frame_height)
            self.last_search_axis = (
                "vertical" if vertical_ratio > horizontal_ratio else "horizontal"
            )
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
    ) -> Optional[SearchDecision]:
        """Return None for ordinary following, otherwise a bounded search decision."""
        if not self.enabled:
            return None

        if not self.searching:
            if self._fresh_target(result):
                return None
            self._start_search(now, height_cm)

        if self._fresh_target(result):
            if self._reacquire_tracker.observe(result):
                self.reset()
                return SearchDecision(RCCommand(), "reacquired", "TARGET_REACQUIRED")
            return SearchDecision(
                RCCommand(),
                "search",
                "REACQUIRE_VERIFY",
                f"ReID {self._reacquire_tracker.progress}",
            )
        self._reacquire_tracker.reset()

        if self.search_started_at is not None and now - self.search_started_at >= self.total_timeout_seconds:
            self.state = "SEARCH_TIMEOUT_LANDING"
            return SearchDecision(RCCommand(), "land", self.state, "search timeout")

        if self.phase_started_at is None:
            self.phase_started_at = now
        elapsed = now - self.phase_started_at

        if self.state == "LOST_HOLD":
            if elapsed < self.hold_seconds:
                return SearchDecision(RCCommand(), "search", self.state, "brief hold")
            if self._looks_too_close() and self.close_recovery_enabled:
                self._enter("CLOSE_BACKOFF", now)
            else:
                self._enter("SEARCH_LAST_DIRECTION", now)
            elapsed = 0.0

        if self.state == "CLOSE_BACKOFF":
            if elapsed < self.close_pulse_seconds:
                return SearchDecision(
                    RCCommand(forward_backward=-self.close_backward_speed),
                    "search",
                    self.state,
                    "bounded rear-clearance recovery",
                )
            self._enter("CLOSE_BACKOFF_PAUSE", now)
            return SearchDecision(RCCommand(), "search", self.state, "recheck target")

        if self.state == "CLOSE_BACKOFF_PAUSE":
            if elapsed < self.close_pause_seconds:
                return SearchDecision(RCCommand(), "search", self.state, "recheck target")
            self.close_attempts += 1
            if self.close_attempts < self.close_max_attempts:
                self._enter("CLOSE_BACKOFF", now)
            else:
                self._enter("SEARCH_LAST_DIRECTION", now)
            return SearchDecision(RCCommand(), "search", self.state)

        if self.state == "SEARCH_LAST_DIRECTION":
            if elapsed < self.last_direction_seconds:
                if self.last_search_axis == "vertical" and self.last_vertical_direction:
                    return SearchDecision(
                        RCCommand(up_down=self.vertical_speed * self.last_vertical_direction),
                        "search",
                        self.state,
                        "last vertical direction",
                    )
                return SearchDecision(
                    RCCommand(yaw=self.yaw_speed * self.last_horizontal_direction),
                    "search",
                    self.state,
                    "last horizontal direction",
                )
            self._enter("SWEEP_CURRENT_SHORT", now)
            return SearchDecision(RCCommand(), "search", self.state, "start yaw sweep")

        if self.state == "SWEEP_CURRENT_SHORT":
            if elapsed < self.sweep_short_seconds:
                return self._yaw_decision(-self.last_horizontal_direction)
            self._enter("SWEEP_CURRENT_LONG", now)
            return SearchDecision(RCCommand(), "search", self.state, "reverse yaw sweep")

        if self.state == "SWEEP_CURRENT_LONG":
            if elapsed < self.sweep_long_seconds:
                return self._yaw_decision(self.last_horizontal_direction)
            self._enter("MOVE_UP", now)
            return SearchDecision(RCCommand(), "search", self.state, "move to upper layer")

        if self.state == "MOVE_UP":
            target = min(self.max_height_cm, (self.search_height_cm or 150) + self.height_step_cm)
            if height_cm is None:
                return SearchDecision(RCCommand(), "search", self.state, "waiting for TOF")
            if height_cm < target - 5:
                return SearchDecision(RCCommand(up_down=self.vertical_speed), "search", self.state)
            self._enter("SWEEP_UP_SHORT", now)
            return SearchDecision(RCCommand(), "search", self.state, "upper layer reached")

        if self.state == "SWEEP_UP_SHORT":
            if elapsed < self.sweep_short_seconds:
                return self._yaw_decision(-self.last_horizontal_direction)
            self._enter("SWEEP_UP_LONG", now)
            return SearchDecision(RCCommand(), "search", self.state, "reverse yaw sweep")

        if self.state == "SWEEP_UP_LONG":
            if elapsed < self.sweep_long_seconds:
                return self._yaw_decision(self.last_horizontal_direction)
            self._enter("MOVE_DOWN", now)
            return SearchDecision(RCCommand(), "search", self.state, "move to lower layer")

        if self.state == "MOVE_DOWN":
            target = max(self.min_height_cm, (self.search_height_cm or 150) - self.height_step_cm)
            if height_cm is None:
                return SearchDecision(RCCommand(), "search", self.state, "waiting for TOF")
            if height_cm > target + 5:
                return SearchDecision(RCCommand(up_down=-self.vertical_speed), "search", self.state)
            self._enter("SWEEP_DOWN_SHORT", now)
            return SearchDecision(RCCommand(), "search", self.state, "lower layer reached")

        if self.state == "SWEEP_DOWN_SHORT":
            if elapsed < self.sweep_short_seconds:
                return self._yaw_decision(-self.last_horizontal_direction)
            self._enter("SWEEP_DOWN_LONG", now)
            return SearchDecision(RCCommand(), "search", self.state, "reverse yaw sweep")

        if self.state == "SWEEP_DOWN_LONG":
            if elapsed < self.sweep_long_seconds:
                return self._yaw_decision(self.last_horizontal_direction)
            self._enter("RETURN_BASE_HEIGHT", now)
            return SearchDecision(RCCommand(), "search", self.state, "return to search height")

        if self.state == "RETURN_BASE_HEIGHT":
            target = self.search_height_cm or 150
            if height_cm is None:
                return SearchDecision(RCCommand(), "search", self.state, "waiting for TOF")
            if height_cm < target - 5:
                return SearchDecision(RCCommand(up_down=self.vertical_speed), "search", self.state)
            if height_cm > target + 5:
                return SearchDecision(RCCommand(up_down=-self.vertical_speed), "search", self.state)
            self._enter("SWEEP_CURRENT_SHORT", now)
            return SearchDecision(RCCommand(), "search", self.state, "repeat sweep")

        return SearchDecision(RCCommand(), "search", self.state)

    def _start_search(self, now: float, height_cm: Optional[int]) -> None:
        self.search_started_at = now
        self.phase_started_at = now
        self.search_height_cm = height_cm
        self.close_attempts = 0
        self._reacquire_tracker.reset()
        self.state = "LOST_HOLD"

    def _looks_too_close(self) -> bool:
        growing = (
            len(self.area_history) >= 2
            and self.area_history[-1] > self.area_history[0] * 1.08
        )
        clipped = self._last_bbox_touches_edge()
        # FollowController 的负前后指令代表它已经判断目标过近并在拉开距离。
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

    def _yaw_decision(self, direction: int) -> SearchDecision:
        return SearchDecision(
            RCCommand(yaw=self.yaw_speed * direction),
            "search",
            self.state,
            "yaw sweep",
        )

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
