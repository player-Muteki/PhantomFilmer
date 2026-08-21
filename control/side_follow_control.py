"""Body-orientation based side-follow control without obstacle arbitration."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Dict, Optional

from control.follow_control import FollowController, RCCommand


@dataclass(frozen=True)
class SideFollowConfig:
    """Tunable parameters for selecting and holding a person's side view."""

    enabled: bool = False
    orientation_stable_frames: int = 5
    orientation_max_deviation_deg: float = 18.0
    minimum_detection_confidence: float = 0.30
    minimum_match_iou: float = 0.20
    angle_tolerance_deg: float = 12.0
    angle_exit_tolerance_deg: float = 20.0
    orbit_entry_frames: int = 3
    lock_stable_frames: int = 6
    tracking_lateral_kp: float = 55.0
    minimum_tracking_lateral_speed: int = 8
    maximum_tracking_lateral_speed: int = 20
    tracking_lateral_direction_sign: int = 1
    lateral_kp: float = 0.22
    minimum_lateral_speed: int = 8
    maximum_lateral_speed: int = 20
    clockwise_lateral_sign: int = -1
    tie_break_target_angle: int = 90
    max_orbit_seconds: float = 20.0

    @classmethod
    def from_config(cls, config: Dict[str, object]) -> "SideFollowConfig":
        section = config.get("side_follow", {}) if isinstance(config, dict) else {}
        if not isinstance(section, dict):
            section = {}

        def as_int(key: str, default: int, minimum: int = 1) -> int:
            try:
                return max(minimum, int(section.get(key, default)))
            except (TypeError, ValueError):
                return default

        def as_float(key: str, default: float, minimum: float = 0.0) -> float:
            try:
                return max(minimum, float(section.get(key, default)))
            except (TypeError, ValueError):
                return default

        clockwise_direction = str(
            section.get("clockwise_lateral_direction", "left")
        ).strip().lower()
        clockwise_sign = 1 if clockwise_direction == "right" else -1
        tracking_sign = (
            -1
            if as_int("tracking_lateral_direction_sign", 1, -1) < 0
            else 1
        )
        tie_angle = 270 if as_int("tie_break_target_angle", 90) == 270 else 90
        tolerance = min(45.0, as_float("angle_tolerance_deg", 12.0))
        exit_tolerance = max(
            tolerance,
            min(90.0, as_float("angle_exit_tolerance_deg", 20.0)),
        )
        minimum_speed = as_int("minimum_lateral_speed", 8, 0)
        maximum_speed = max(
            minimum_speed, as_int("maximum_lateral_speed", 20, 1)
        )
        minimum_tracking_speed = as_int(
            "minimum_tracking_lateral_speed", 8, 0
        )
        maximum_tracking_speed = max(
            minimum_tracking_speed,
            as_int("maximum_tracking_lateral_speed", 20, 1),
        )
        return cls(
            enabled=bool(section.get("enabled", False)),
            orientation_stable_frames=as_int("orientation_stable_frames", 5),
            orientation_max_deviation_deg=min(
                90.0, as_float("orientation_max_deviation_deg", 18.0)
            ),
            minimum_detection_confidence=min(
                1.0, as_float("minimum_detection_confidence", 0.30)
            ),
            minimum_match_iou=min(1.0, as_float("minimum_match_iou", 0.20)),
            angle_tolerance_deg=tolerance,
            angle_exit_tolerance_deg=exit_tolerance,
            orbit_entry_frames=as_int("orbit_entry_frames", 3),
            lock_stable_frames=as_int("lock_stable_frames", 6),
            tracking_lateral_kp=as_float("tracking_lateral_kp", 55.0),
            minimum_tracking_lateral_speed=minimum_tracking_speed,
            maximum_tracking_lateral_speed=maximum_tracking_speed,
            tracking_lateral_direction_sign=tracking_sign,
            lateral_kp=as_float("lateral_kp", 0.22),
            minimum_lateral_speed=minimum_speed,
            maximum_lateral_speed=maximum_speed,
            clockwise_lateral_sign=clockwise_sign,
            tie_break_target_angle=tie_angle,
            max_orbit_seconds=as_float("max_orbit_seconds", 20.0, 1.0),
        )


@dataclass
class SideFollowDebugInfo:
    state: str = "SIDE_SAMPLING"
    current_angle: Optional[float] = None
    selected_angle: Optional[int] = None
    angle_error: Optional[float] = None
    stable_samples: int = 0
    lock_frames: int = 0
    orbit_direction: str = ""


class SideFollowController:
    """Choose the nearer 90/270-degree side once, then orbit to hold it."""

    def __init__(self, follow_controller: FollowController, config: SideFollowConfig):
        self.follow_controller = follow_controller
        self.safety_manager = follow_controller.safety_manager
        self.config = config
        self._samples: Deque[float] = deque(maxlen=config.orientation_stable_frames)
        self._selected_angle: Optional[int] = None
        self._orbit_entry_frames = 0
        self._lock_frames = 0
        self._orbit_active = False
        self._orbit_started_at: Optional[float] = None
        self.last_debug = SideFollowDebugInfo()

    @classmethod
    def from_config(
        cls, follow_controller: FollowController, config: Dict[str, object]
    ) -> "SideFollowController":
        return cls(follow_controller, SideFollowConfig.from_config(config))

    @property
    def selected_angle(self) -> Optional[int]:
        return self._selected_angle

    def reset(self, *, preserve_selection: bool = False) -> None:
        self._samples.clear()
        if not preserve_selection:
            self._selected_angle = None
        self._orbit_entry_frames = 0
        self._lock_frames = 0
        self._orbit_active = False
        self._orbit_started_at = None
        self.last_debug = SideFollowDebugInfo(selected_angle=self._selected_angle)

    def compute_command(
        self,
        target_result: Dict[str, object],
        frame_width: int,
        frame_height: int,
        now: float,
    ) -> RCCommand:
        """Return a bounded side-follow command for one fresh ReID result."""
        angle = self._valid_angle(target_result)
        if angle is None:
            if self._selected_angle is None:
                self._samples.clear()
            self.last_debug = SideFollowDebugInfo(
                state="SIDE_ANGLE_WAIT",
                selected_angle=self._selected_angle,
                stable_samples=len(self._samples),
            )
            return self.follow_controller.hover()

        if self._selected_angle is None:
            self._samples.append(angle)
            average = self._circular_average(self._samples)
            stable = (
                len(self._samples) >= self.config.orientation_stable_frames
                and max(
                    self._circular_distance(sample, average)
                    for sample in self._samples
                )
                <= self.config.orientation_max_deviation_deg
            )
            if not stable:
                self.last_debug = SideFollowDebugInfo(
                    state="SIDE_SAMPLING",
                    current_angle=angle,
                    stable_samples=len(self._samples),
                )
                return self.follow_controller.hover()
            self._selected_angle = self._choose_nearest_side(average)

        error = self._signed_angle_error(float(self._selected_angle), angle)
        self._update_orbit_state(error, now)
        if (
            self._orbit_started_at is not None
            and now - self._orbit_started_at >= self.config.max_orbit_seconds
            and self._orbit_active
        ):
            self.last_debug = SideFollowDebugInfo(
                state="SIDE_TIMEOUT",
                current_angle=angle,
                selected_angle=self._selected_angle,
                angle_error=error,
            )
            return self.follow_controller.hover()

        horizontal_error, forward, vertical = self._tracking_axes(
            target_result, frame_width, frame_height
        )
        tracking_lateral = self._tracking_lateral_command(horizontal_error)
        orbit_lateral = (
            self._orbit_lateral_command(error) if self._orbit_active else 0
        )
        orbit_direction = ""
        if self._orbit_active and error != 0:
            orbit_direction = "CLOCKWISE" if error > 0 else "COUNTERCLOCKWISE"
        lateral = self._clamp(
            tracking_lateral + orbit_lateral,
            -self.config.maximum_lateral_speed,
            self.config.maximum_lateral_speed,
        )
        # 跑道平移阶段不转机头，避免侧拍逐渐变成斜拍。只有绕人时才用偏航
        # 把人物维持在镜头中心，横移则负责恢复相对人物的侧面观察方位。
        yaw = (
            self.follow_controller._compute_yaw(horizontal_error)
            if self._orbit_active
            else 0
        )
        limited = self.safety_manager.limit_rc_command(
            lateral, forward, vertical, yaw
        )
        command = RCCommand(*limited)
        self.last_debug = SideFollowDebugInfo(
            state="SIDE_ORBITING" if self._orbit_active else "SIDE_TRACKING",
            current_angle=angle,
            selected_angle=self._selected_angle,
            angle_error=error,
            stable_samples=len(self._samples),
            lock_frames=self._lock_frames,
            orbit_direction=orbit_direction,
        )
        return command

    def _update_orbit_state(self, error: float, now: float) -> None:
        absolute_error = abs(error)
        if self._orbit_active:
            if absolute_error <= self.config.angle_tolerance_deg:
                self._lock_frames += 1
                if self._lock_frames >= self.config.lock_stable_frames:
                    self._orbit_active = False
                    self._orbit_entry_frames = 0
                    self._lock_frames = 0
                    self._orbit_started_at = None
            else:
                self._lock_frames = 0
            return

        if absolute_error > self.config.angle_exit_tolerance_deg:
            self._orbit_entry_frames += 1
            if self._orbit_entry_frames >= self.config.orbit_entry_frames:
                self._orbit_active = True
                self._lock_frames = 0
                self._orbit_started_at = now
        else:
            self._orbit_entry_frames = 0

    def _valid_angle(self, result: Dict[str, object]) -> Optional[float]:
        if not result.get("found") or bool(result.get("is_predicted", False)):
            return None
        try:
            angle = float(result["body_orientation_angle"]) % 360.0
            confidence = float(
                result.get("body_orientation_detection_confidence") or 0.0
            )
            match_iou = float(result.get("body_orientation_match_iou") or 0.0)
        except (KeyError, TypeError, ValueError):
            return None
        if confidence < self.config.minimum_detection_confidence:
            return None
        if match_iou < self.config.minimum_match_iou:
            return None
        return angle

    def _choose_nearest_side(self, angle: float) -> int:
        distance_90 = self._circular_distance(angle, 90.0)
        distance_270 = self._circular_distance(angle, 270.0)
        if math.isclose(distance_90, distance_270, abs_tol=1e-6):
            return self.config.tie_break_target_angle
        return 90 if distance_90 < distance_270 else 270

    def _orbit_lateral_command(self, error: float) -> int:
        raw_magnitude = int(round(abs(error) * self.config.lateral_kp))
        if raw_magnitude == 0:
            return 0
        magnitude = max(self.config.minimum_lateral_speed, raw_magnitude)
        magnitude = min(self.config.maximum_lateral_speed, magnitude)
        # 现场标定：绕人顺时针会增大 JointBDOE 角度，逆时针会减小。
        # 正误差（目标角 > 当前角）必须顺时针；负误差必须逆时针。
        direction = (
            self.config.clockwise_lateral_sign
            if error > 0
            else -self.config.clockwise_lateral_sign
        )
        return direction * magnitude

    def _tracking_lateral_command(self, horizontal_error: float) -> int:
        if abs(horizontal_error) <= self.follow_controller.horizontal_dead_zone_ratio:
            return 0
        raw = int(round(horizontal_error * self.config.tracking_lateral_kp))
        if raw == 0:
            return 0
        magnitude = max(
            self.config.minimum_tracking_lateral_speed, abs(raw)
        )
        magnitude = min(
            self.config.maximum_tracking_lateral_speed, magnitude
        )
        direction = 1 if raw > 0 else -1
        return (
            direction
            * magnitude
            * self.config.tracking_lateral_direction_sign
        )

    def _tracking_axes(
        self, result: Dict[str, object], frame_width: int, frame_height: int
    ) -> tuple[float, int, int]:
        center = result.get("center")
        if center is None:
            return 0.0, 0, 0
        target_x, target_y = center  # type: ignore[misc]
        horizontal_error = (float(target_x) - frame_width / 2.0) / max(
            1.0, frame_width / 2.0
        )
        vertical_error = (float(target_y) - frame_height / 2.0) / max(
            1.0, frame_height / 2.0
        )
        frame_area = max(1.0, float(frame_width * frame_height))
        area_ratio = float(result.get("area") or 0.0) / frame_area

        follow = self.follow_controller
        vertical = follow._compute_vertical(vertical_error)
        forward = follow._compute_forward(area_ratio)
        if vertical != 0 and forward != 0:
            forward = (
                follow.forward_speed_while_aligning
                if forward > 0
                else -follow.forward_speed_while_aligning
            )
        return horizontal_error, forward, vertical

    @staticmethod
    def _clamp(value: int, lower: int, upper: int) -> int:
        return max(lower, min(upper, int(value)))

    @staticmethod
    def _signed_angle_error(target: float, current: float) -> float:
        return (target - current + 180.0) % 360.0 - 180.0

    @staticmethod
    def _circular_distance(first: float, second: float) -> float:
        return abs(SideFollowController._signed_angle_error(first, second))

    @staticmethod
    def _circular_average(samples: Deque[float]) -> float:
        sine = sum(math.sin(math.radians(value)) for value in samples)
        cosine = sum(math.cos(math.radians(value)) for value in samples)
        return math.degrees(math.atan2(sine, cosine)) % 360.0
