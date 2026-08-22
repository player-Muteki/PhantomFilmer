"""Body-orientation based side-follow control without obstacle arbitration."""

import math
from collections import deque
from dataclasses import dataclass

from control.follow_control import FollowController, RCCommand


@dataclass(frozen=True)
class SideFollowConfig:
    """Tunable parameters for selecting and holding a person's side view."""

    enabled: bool = False
    orientation_stable_frames: int = 5
    orientation_max_deviation_deg: float = 18.0
    minimum_detection_confidence: float = 0.30
    minimum_match_iou: float = 0.20
    lost_confirm_seconds: float = 0.50
    angle_tolerance_deg: float = 12.0
    angle_exit_tolerance_deg: float = 20.0
    orbit_entry_frames: int = 2
    lock_stable_frames: int = 6
    centered_turn_stable_frames: int = 5
    centered_turn_max_deviation_deg: float = 8.0
    center_tolerance_scale: float = 0.70
    distance_area_scale: float = 1.0
    tracking_lateral_kp: float = 70.0
    minimum_tracking_lateral_speed: int = 8
    maximum_tracking_lateral_speed: int = 25
    tracking_lateral_direction_sign: int = 1
    lateral_kp: float = 0.35
    minimum_lateral_speed: int = 10
    maximum_lateral_speed: int = 25
    clockwise_lateral_sign: int = -1
    orbit_yaw_feedforward_gain: float = 0.80
    maximum_orbit_yaw_speed: int = 30
    tie_break_target_angle: int = 90

    @classmethod
    def from_config(cls, config: dict[str, object]) -> "SideFollowConfig":
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

        clockwise_direction = (
            str(section.get("clockwise_lateral_direction", "left")).strip().lower()
        )
        clockwise_sign = 1 if clockwise_direction == "right" else -1
        tracking_sign = (
            -1 if as_int("tracking_lateral_direction_sign", 1, -1) < 0 else 1
        )
        tie_angle = 270 if as_int("tie_break_target_angle", 90) == 270 else 90
        tolerance = min(45.0, as_float("angle_tolerance_deg", 12.0))
        exit_tolerance = max(
            tolerance,
            min(90.0, as_float("angle_exit_tolerance_deg", 20.0)),
        )
        minimum_speed = as_int("minimum_lateral_speed", 10, 0)
        maximum_speed = max(minimum_speed, as_int("maximum_lateral_speed", 25, 1))
        minimum_tracking_speed = as_int("minimum_tracking_lateral_speed", 8, 0)
        maximum_tracking_speed = max(
            minimum_tracking_speed,
            as_int("maximum_tracking_lateral_speed", 25, 1),
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
            lost_confirm_seconds=as_float("lost_confirm_seconds", 0.50),
            angle_tolerance_deg=tolerance,
            angle_exit_tolerance_deg=exit_tolerance,
            orbit_entry_frames=as_int("orbit_entry_frames", 2),
            lock_stable_frames=as_int("lock_stable_frames", 6),
            centered_turn_stable_frames=as_int("centered_turn_stable_frames", 5),
            centered_turn_max_deviation_deg=min(
                45.0, as_float("centered_turn_max_deviation_deg", 8.0)
            ),
            center_tolerance_scale=min(1.0, as_float("center_tolerance_scale", 0.70)),
            distance_area_scale=min(1.0, as_float("distance_area_scale", 1.0, 0.25)),
            tracking_lateral_kp=as_float("tracking_lateral_kp", 70.0),
            minimum_tracking_lateral_speed=minimum_tracking_speed,
            maximum_tracking_lateral_speed=maximum_tracking_speed,
            tracking_lateral_direction_sign=tracking_sign,
            lateral_kp=as_float("lateral_kp", 0.35),
            minimum_lateral_speed=minimum_speed,
            maximum_lateral_speed=maximum_speed,
            clockwise_lateral_sign=clockwise_sign,
            orbit_yaw_feedforward_gain=as_float("orbit_yaw_feedforward_gain", 0.80),
            maximum_orbit_yaw_speed=as_int("maximum_orbit_yaw_speed", 30),
            tie_break_target_angle=tie_angle,
        )


@dataclass
class SideFollowDebugInfo:
    state: str = "SIDE_SAMPLING"
    current_angle: float | None = None
    selected_angle: int | None = None
    angle_error: float | None = None
    stable_samples: int = 0
    lock_frames: int = 0
    orbit_direction: str = ""
    yaw_feedforward: int = 0
    yaw_feedback: int = 0
    side_locked: bool = False
    position_priority: bool = False
    side_reselect_pending: bool = False
    centered_angle_samples: int = 0
    centered_angle_stable: bool = False
    center_tolerance_ratio: float = 0.0
    orbit_active: bool = False
    horizontal_error: float | None = None
    tracking_lateral: int = 0
    orbit_lateral: int = 0


class SideFollowController:
    """Hold one configured body-view angle with side-follow motion logic."""

    def __init__(
        self,
        follow_controller: FollowController,
        config: SideFollowConfig,
        *,
        target_angles: tuple[int, ...] = (90, 270),
        distance_area_scale: float | None = None,
    ):
        self.follow_controller = follow_controller
        self.safety_manager = follow_controller.safety_manager
        self.config = config
        requested_distance_scale = (
            config.distance_area_scale
            if distance_area_scale is None
            else distance_area_scale
        )
        self.distance_area_scale = max(0.25, min(1.0, float(requested_distance_scale)))
        normalized_targets = tuple(dict.fromkeys(int(angle) % 360 for angle in target_angles))
        if not normalized_targets:
            raise ValueError("target_angles must contain at least one angle")
        self.target_angles = normalized_targets
        self._samples: deque[float] = deque(maxlen=config.orientation_stable_frames)
        self._centered_angle_samples: deque[float] = deque(
            maxlen=config.centered_turn_stable_frames
        )
        self._selected_angle: int | None = None
        self._orbit_entry_frames = 0
        self._lock_frames = 0
        self._orbit_active = False
        self._side_locked_once = False
        self._side_reselect_pending = False
        self.last_debug = SideFollowDebugInfo()

    @classmethod
    def from_config(
        cls,
        follow_controller: FollowController,
        config: dict[str, object],
        *,
        target_angles: tuple[int, ...] = (90, 270),
        distance_area_scale: float | None = None,
    ) -> "SideFollowController":
        return cls(
            follow_controller,
            SideFollowConfig.from_config(config),
            target_angles=target_angles,
            distance_area_scale=distance_area_scale,
        )

    @property
    def selected_angle(self) -> int | None:
        return self._selected_angle

    def reset(self, *, preserve_selection: bool = False) -> None:
        self._samples.clear()
        self._centered_angle_samples.clear()
        if not preserve_selection:
            self._selected_angle = None
            self._side_locked_once = False
        self._orbit_entry_frames = 0
        self._lock_frames = 0
        self._orbit_active = False
        self._side_reselect_pending = False
        self.last_debug = SideFollowDebugInfo(selected_angle=self._selected_angle)

    def compute_command(
        self,
        target_result: dict[str, object],
        frame_width: int,
        frame_height: int,
        now: float,
    ) -> RCCommand:
        """Return a bounded side-follow command for one fresh ReID result."""
        horizontal_error, forward, vertical = self._tracking_axes(
            target_result, frame_width, frame_height
        )
        tracking_lateral = self._tracking_lateral_command(horizontal_error)
        angle = self._valid_angle(target_result)
        if angle is None:
            if self._selected_angle is None:
                self._samples.clear()
            elif self._side_locked_once and tracking_lateral != 0:
                return self._position_priority_command(
                    tracking_lateral,
                    forward,
                    vertical,
                    angle=None,
                    horizontal_error=horizontal_error,
                )
            self.last_debug = SideFollowDebugInfo(
                state="SIDE_ANGLE_WAIT",
                selected_angle=self._selected_angle,
                stable_samples=len(self._samples),
                side_locked=self._side_locked_once,
                side_reselect_pending=self._side_reselect_pending,
                center_tolerance_ratio=self._center_tolerance_ratio(),
                horizontal_error=horizontal_error,
                tracking_lateral=tracking_lateral,
            )
            return self.follow_controller.hover()

        if self._selected_angle is None:
            self._samples.append(angle)
            average = self._circular_average(self._samples)
            stable = (
                len(self._samples) >= self.config.orientation_stable_frames
                and max(
                    self._circular_distance(sample, average) for sample in self._samples
                )
                <= self.config.orientation_max_deviation_deg
            )
            if not stable:
                self.last_debug = SideFollowDebugInfo(
                    state="SIDE_SAMPLING",
                    current_angle=angle,
                    stable_samples=len(self._samples),
                    center_tolerance_ratio=self._center_tolerance_ratio(),
                    horizontal_error=horizontal_error,
                    tracking_lateral=tracking_lateral,
                )
                return self.follow_controller.hover()
            self._selected_angle = self._choose_nearest_target(average)

        # Once the first side has been locked, keeping the person centered owns
        # the lateral axis. Any active orbit is cancelled instead of being
        # summed with the position command.
        if self._side_locked_once and tracking_lateral != 0:
            return self._position_priority_command(
                tracking_lateral,
                forward,
                vertical,
                angle=angle,
                horizontal_error=horizontal_error,
            )

        # Once the target is centered, wait for body orientation to settle over
        # several frames. Only then reselect the nearest 90/270 side and decide
        # whether an orbit is actually needed. During an active orbit the chosen
        # side stays fixed until the maneuver is complete.
        centered_angle_stable = False
        if self._side_locked_once and not self._orbit_active:
            stable_angle = self._stable_centered_angle(angle)
            if stable_angle is None:
                self._orbit_entry_frames = 0
                self._lock_frames = 0
                return self._centered_turn_wait_command(
                    forward,
                    vertical,
                    angle=angle,
                    horizontal_error=horizontal_error,
                    tracking_lateral=tracking_lateral,
                )
            centered_angle_stable = True
            self._selected_angle = self._choose_nearest_target(stable_angle)
            self._side_reselect_pending = False

        error = self._signed_angle_error(float(self._selected_angle), angle)
        orbit_was_active = self._orbit_active
        self._update_orbit_state(error)
        if self._orbit_active != orbit_was_active:
            self._centered_angle_samples.clear()

        orbit_lateral = self._orbit_lateral_command(error) if self._orbit_active else 0
        orbit_direction = ""
        if self._orbit_active and error != 0:
            orbit_direction = "CLOCKWISE" if error > 0 else "COUNTERCLOCKWISE"
        lateral = self._clamp(
            tracking_lateral + orbit_lateral,
            -self.config.maximum_lateral_speed,
            self.config.maximum_lateral_speed,
        )
        # 跑道平移阶段不转机头。绕人时根据绕行方向立即加入偏航前馈，随后
        # 再叠加人物中心误差反馈，避免高速横移后才开始追赶画面偏移。
        yaw_feedback = 0
        yaw_feedforward = 0
        if self._orbit_active:
            yaw_feedback = self.follow_controller._compute_yaw(horizontal_error)
            yaw_feedforward = self._orbit_yaw_feedforward(error, orbit_lateral)
        yaw = self._clamp(
            yaw_feedforward + yaw_feedback,
            -self.config.maximum_orbit_yaw_speed,
            self.config.maximum_orbit_yaw_speed,
        )
        limited = self.safety_manager.limit_rc_command(lateral, forward, vertical, yaw)
        command = RCCommand(*limited)
        self.last_debug = SideFollowDebugInfo(
            state="SIDE_ORBITING" if self._orbit_active else "SIDE_TRACKING",
            current_angle=angle,
            selected_angle=self._selected_angle,
            angle_error=error,
            stable_samples=len(self._samples),
            lock_frames=self._lock_frames,
            orbit_direction=orbit_direction,
            yaw_feedforward=yaw_feedforward,
            yaw_feedback=yaw_feedback,
            side_locked=self._side_locked_once,
            side_reselect_pending=self._side_reselect_pending,
            centered_angle_samples=len(self._centered_angle_samples),
            centered_angle_stable=centered_angle_stable,
            center_tolerance_ratio=self._center_tolerance_ratio(),
            orbit_active=self._orbit_active,
            horizontal_error=horizontal_error,
            tracking_lateral=tracking_lateral,
            orbit_lateral=orbit_lateral,
        )
        return command

    def _position_priority_command(
        self,
        lateral: int,
        forward: int,
        vertical: int,
        *,
        angle: float | None,
        horizontal_error: float,
    ) -> RCCommand:
        """Return a yaw-free centering command and defer all angle decisions."""
        self._orbit_active = False
        self._orbit_entry_frames = 0
        self._lock_frames = 0
        self._side_reselect_pending = True
        self._centered_angle_samples.clear()
        limited = self.safety_manager.limit_rc_command(lateral, forward, vertical, 0)
        command = RCCommand(*limited)
        error = (
            None
            if angle is None or self._selected_angle is None
            else self._signed_angle_error(float(self._selected_angle), angle)
        )
        self.last_debug = SideFollowDebugInfo(
            state="SIDE_POSITION_TRACKING",
            current_angle=angle,
            selected_angle=self._selected_angle,
            angle_error=error,
            stable_samples=len(self._samples),
            side_locked=True,
            position_priority=True,
            side_reselect_pending=True,
            centered_angle_samples=0,
            center_tolerance_ratio=self._center_tolerance_ratio(),
            horizontal_error=horizontal_error,
            tracking_lateral=lateral,
        )
        return command

    def _centered_turn_wait_command(
        self,
        forward: int,
        vertical: int,
        *,
        angle: float,
        horizontal_error: float,
        tracking_lateral: int,
    ) -> RCCommand:
        """Hold yaw/lateral while collecting a stable centered body angle."""
        limited = self.safety_manager.limit_rc_command(0, forward, vertical, 0)
        command = RCCommand(*limited)
        error = (
            None
            if self._selected_angle is None
            else self._signed_angle_error(float(self._selected_angle), angle)
        )
        self.last_debug = SideFollowDebugInfo(
            state="SIDE_TURN_STABILIZING",
            current_angle=angle,
            selected_angle=self._selected_angle,
            angle_error=error,
            stable_samples=len(self._samples),
            side_locked=True,
            side_reselect_pending=self._side_reselect_pending,
            centered_angle_samples=len(self._centered_angle_samples),
            centered_angle_stable=False,
            center_tolerance_ratio=self._center_tolerance_ratio(),
            horizontal_error=horizontal_error,
            tracking_lateral=tracking_lateral,
        )
        return command

    def _stable_centered_angle(self, angle: float) -> float | None:
        """Return a circular average only after the centered angle settles."""
        self._centered_angle_samples.append(angle)
        if len(self._centered_angle_samples) < self.config.centered_turn_stable_frames:
            return None
        average = self._circular_average(self._centered_angle_samples)
        maximum_deviation = max(
            self._circular_distance(sample, average)
            for sample in self._centered_angle_samples
        )
        if maximum_deviation > self.config.centered_turn_max_deviation_deg:
            return None
        return average

    def _update_orbit_state(self, error: float) -> None:
        absolute_error = abs(error)
        if self._orbit_active:
            if absolute_error <= self.config.angle_tolerance_deg:
                self._lock_frames += 1
                if self._lock_frames >= self.config.lock_stable_frames:
                    self._orbit_active = False
                    self._side_locked_once = True
                    self._orbit_entry_frames = 0
                    self._lock_frames = 0
            else:
                self._lock_frames = 0
            return

        if (
            not self._side_locked_once
            and absolute_error <= self.config.angle_tolerance_deg
        ):
            self._lock_frames += 1
            if self._lock_frames >= self.config.lock_stable_frames:
                self._side_locked_once = True
                self._lock_frames = 0
            self._orbit_entry_frames = 0
            return

        if not self._side_locked_once:
            self._lock_frames = 0

        if absolute_error > self.config.angle_exit_tolerance_deg:
            self._orbit_entry_frames += 1
            if self._orbit_entry_frames >= self.config.orbit_entry_frames:
                self._orbit_active = True
                self._lock_frames = 0
        else:
            self._orbit_entry_frames = 0

    def _valid_angle(self, result: dict[str, object]) -> float | None:
        if not result.get("found") or bool(result.get("is_predicted", False)):
            return None
        angle_value = self._number(result.get("body_orientation_angle"))
        confidence = self._number(result.get("body_orientation_detection_confidence"))
        match_iou = self._number(result.get("body_orientation_match_iou"))
        if angle_value is None or confidence is None or match_iou is None:
            return None
        angle = angle_value % 360.0
        if confidence < self.config.minimum_detection_confidence:
            return None
        if match_iou < self.config.minimum_match_iou:
            return None
        return angle

    def _choose_nearest_target(self, angle: float) -> int:
        """Choose the configured view requiring the least circular travel."""
        distances = {
            target: self._circular_distance(angle, float(target))
            for target in self.target_angles
        }
        minimum = min(distances.values())
        nearest = [
            target
            for target, distance in distances.items()
            if math.isclose(distance, minimum, abs_tol=1e-6)
        ]
        if self.config.tie_break_target_angle in nearest:
            return self.config.tie_break_target_angle
        return nearest[0]

    def _orbit_lateral_command(self, error: float) -> int:
        raw_magnitude = round(abs(error) * self.config.lateral_kp)
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
        if abs(horizontal_error) <= self._center_tolerance_ratio():
            return 0
        raw = round(horizontal_error * self.config.tracking_lateral_kp)
        if raw == 0:
            return 0
        magnitude = max(self.config.minimum_tracking_lateral_speed, abs(raw))
        magnitude = min(self.config.maximum_tracking_lateral_speed, magnitude)
        direction = 1 if raw > 0 else -1
        return direction * magnitude * self.config.tracking_lateral_direction_sign

    def _center_tolerance_ratio(self) -> float:
        """Return the side-follow center tolerance after the 30% reduction."""
        return (
            float(self.follow_controller.horizontal_dead_zone_ratio)
            * self.config.center_tolerance_scale
        )

    def _orbit_yaw_feedforward(self, error: float, orbit_lateral: int) -> int:
        if error == 0 or orbit_lateral == 0:
            return 0
        magnitude = round(abs(orbit_lateral) * self.config.orbit_yaw_feedforward_gain)
        # Tello 正 yaw 为顺时针、负 yaw 为逆时针；该方向与需要增大或
        # 减小人体角度的绕行方向一致，不依赖横移通道的左右标定。
        return magnitude if error > 0 else -magnitude

    def _tracking_axes(
        self, result: dict[str, object], frame_width: int, frame_height: int
    ) -> tuple[float, int, int]:
        center = result.get("center")
        if not isinstance(center, (tuple, list)) or len(center) != 2:
            return 0.0, 0, 0
        target_x = self._number(center[0])
        target_y = self._number(center[1])
        if target_x is None or target_y is None:
            return 0.0, 0, 0
        horizontal_error = (target_x - frame_width / 2.0) / max(1.0, frame_width / 2.0)
        vertical_error = (target_y - frame_height / 2.0) / max(1.0, frame_height / 2.0)
        frame_area = max(1.0, float(frame_width * frame_height))
        area_ratio = (self._number(result.get("area")) or 0.0) / frame_area

        follow = self.follow_controller
        vertical = follow._compute_vertical(vertical_error)
        forward = self._compute_distance_forward(area_ratio)
        if vertical != 0 and forward != 0:
            forward = (
                follow.forward_speed_while_aligning
                if forward > 0
                else -follow.forward_speed_while_aligning
            )
        return horizontal_error, forward, vertical

    def _compute_distance_forward(self, area_ratio: float) -> int:
        """Keep a side-profile target at the calibrated physical distance.

        A side-view body produces a smaller ReID box than the same person viewed
        from the front. Scaling only the side controller's area band prevents it
        from closing in merely to match the normal-follow pixel area target.
        """
        follow = self.follow_controller
        target_min = follow.target_area_ratio_min * self.distance_area_scale
        target_max = follow.target_area_ratio_max * self.distance_area_scale
        if area_ratio < target_min:
            return follow.maximum_forward_speed
        if area_ratio > target_max:
            return -follow.maximum_forward_speed
        return 0

    @staticmethod
    def _number(value: object) -> float | None:
        """Return one finite numeric payload field, or ``None`` when malformed."""
        if not isinstance(value, (int, float, str)):
            return None
        try:
            number = float(value)
        except ValueError:
            return None
        return number if math.isfinite(number) else None

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
    def _circular_average(samples: deque[float]) -> float:
        sine = sum(math.sin(math.radians(value)) for value in samples)
        cosine = sum(math.cos(math.radians(value)) for value in samples)
        return math.degrees(math.atan2(sine, cosine)) % 360.0
