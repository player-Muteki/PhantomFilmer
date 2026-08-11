"""Low-speed target-following control for red target tracking."""

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from drone.safety import SafetyManager


@dataclass
class RCCommand:
    """Remote-control command values for Tello-style velocity control."""

    left_right: int = 0
    forward_backward: int = 0
    up_down: int = 0
    yaw: int = 0

    def as_tuple(self) -> Tuple[int, int, int, int]:
        """Return the command in DroneAdapter.move_rc order."""
        return (self.left_right, self.forward_backward, self.up_down, self.yaw)


@dataclass
class FollowDebugInfo:
    """Debug values from the latest follow-control calculation."""

    target_center_x: Optional[int] = None
    target_center_y: Optional[int] = None
    frame_center_x: Optional[int] = None
    frame_center_y: Optional[int] = None
    horizontal_error: int = 0
    horizontal_error_ratio: float = 0.0
    vertical_error: int = 0
    vertical_error_ratio: float = 0.0
    target_area: float = 0.0
    area_ratio: float = 0.0
    target_state: str = "LOST"
    yaw: int = 0
    left_right: int = 0
    forward_backward: int = 0
    up_down: int = 0


class FollowController:
    """Convert target detection results into safe low-speed RC commands."""

    def __init__(
        self,
        safety_manager: SafetyManager,
        horizontal_dead_zone_ratio: float = 0.08,
        yaw_kp: float = 80.0,
        minimum_yaw_speed: int = 10,
        maximum_yaw_speed: int = 25,
        target_area_ratio_min: float = 0.015,
        target_area_ratio_max: float = 0.060,
        forward_kp: float = 500.0,
        minimum_forward_speed: int = 10,
        maximum_forward_speed: int = 20,
        forward_speed_while_aligning: int = 20,
        large_horizontal_error_ratio: float = 0.28,
        forward_speed_while_turning_ratio: float = 0.25,
        vertical_dead_zone_ratio: float = 0.10,
        vertical_speed: int = 8,
        target_lock_stable_frames: int = 15,
        target_lock_exit_area_ratio_min: Optional[float] = None,
        target_lock_exit_area_ratio_max: Optional[float] = None,
        target_lock_exit_horizontal_dead_zone_ratio: Optional[float] = None,
        target_lock_exit_vertical_dead_zone_ratio: Optional[float] = None,
    ) -> None:
        self.safety_manager = safety_manager
        self.horizontal_dead_zone_ratio = self._clamp_float(horizontal_dead_zone_ratio, 0.0, 0.5, 0.08)
        self.yaw_kp = self._positive_float(yaw_kp, 80.0)
        self.minimum_yaw_speed = self._non_negative_int(minimum_yaw_speed, 10)
        self.maximum_yaw_speed = self._positive_int(maximum_yaw_speed, 25)
        self.target_area_ratio_min = self._clamp_float(target_area_ratio_min, 0.0001, 1.0, 0.015)
        self.target_area_ratio_max = self._clamp_float(
            target_area_ratio_max, self.target_area_ratio_min, 1.0, 0.060
        )
        self.forward_kp = self._positive_float(forward_kp, 500.0)
        self.minimum_forward_speed = self._non_negative_int(minimum_forward_speed, 10)
        self.maximum_forward_speed = self._positive_int(maximum_forward_speed, 20)
        self.forward_speed_while_aligning = min(
            self._non_negative_int(forward_speed_while_aligning, 20), self.maximum_forward_speed
        )
        self.large_horizontal_error_ratio = self._clamp_float(
            large_horizontal_error_ratio, self.horizontal_dead_zone_ratio, 1.0, 0.28
        )
        self.forward_speed_while_turning_ratio = self._clamp_float(
            forward_speed_while_turning_ratio, 0.0, 1.0, 0.25
        )
        self.vertical_dead_zone_ratio = self._clamp_float(vertical_dead_zone_ratio, 0.0, 0.5, 0.10)
        self.vertical_speed = self._non_negative_int(vertical_speed, 8)
        self.target_lock_stable_frames = self._positive_int(target_lock_stable_frames, 15)
        self.target_lock_exit_area_ratio_min = self._clamp_float(
            target_lock_exit_area_ratio_min,
            0.0001,
            self.target_area_ratio_min,
            self.target_area_ratio_min,
        )
        self.target_lock_exit_area_ratio_max = self._clamp_float(
            target_lock_exit_area_ratio_max,
            self.target_area_ratio_max,
            1.0,
            self.target_area_ratio_max,
        )
        self.target_lock_exit_horizontal_dead_zone_ratio = self._clamp_float(
            target_lock_exit_horizontal_dead_zone_ratio,
            self.horizontal_dead_zone_ratio,
            1.0,
            self.horizontal_dead_zone_ratio,
        )
        self.target_lock_exit_vertical_dead_zone_ratio = self._clamp_float(
            target_lock_exit_vertical_dead_zone_ratio,
            self.vertical_dead_zone_ratio,
            1.0,
            self.vertical_dead_zone_ratio,
        )
        self._target_locked = False
        self._stable_frame_count = 0
        self.last_debug = FollowDebugInfo()

    @classmethod
    def from_config(cls, safety_manager: SafetyManager, config: Dict[str, object]) -> "FollowController":
        """Build a controller from config.yaml with safe defaults."""
        return cls(
            safety_manager=safety_manager,
            horizontal_dead_zone_ratio=cls._config_float(config, "horizontal_dead_zone_ratio", 0.08),
            yaw_kp=cls._config_float(config, "yaw_kp", 80.0),
            minimum_yaw_speed=cls._config_int(config, "minimum_yaw_speed", 10),
            maximum_yaw_speed=cls._config_int(config, "maximum_yaw_speed", 25),
            target_area_ratio_min=cls._config_float(config, "target_area_ratio_min", 0.015),
            target_area_ratio_max=cls._config_float(config, "target_area_ratio_max", 0.060),
            forward_kp=cls._config_float(config, "forward_kp", 500.0),
            minimum_forward_speed=cls._config_int(config, "minimum_forward_speed", 10),
            maximum_forward_speed=cls._config_int(config, "maximum_forward_speed", 20),
            forward_speed_while_aligning=cls._config_int(config, "forward_speed_while_aligning", 20),
            large_horizontal_error_ratio=cls._config_float(config, "large_horizontal_error_ratio", 0.28),
            forward_speed_while_turning_ratio=cls._config_float(
                config, "forward_speed_while_turning_ratio", 0.25
            ),
            vertical_dead_zone_ratio=cls._config_float(config, "vertical_dead_zone_ratio", 0.10),
            vertical_speed=cls._config_int(config, "vertical_speed", 8),
            target_lock_stable_frames=cls._config_int(config, "target_lock_stable_frames", 15),
            target_lock_exit_area_ratio_min=cls._config_float(
                config, "target_lock_exit_area_ratio_min", 0.015
            ),
            target_lock_exit_area_ratio_max=cls._config_float(
                config, "target_lock_exit_area_ratio_max", 0.060
            ),
            target_lock_exit_horizontal_dead_zone_ratio=cls._config_float(
                config, "target_lock_exit_horizontal_dead_zone_ratio", 0.08
            ),
            target_lock_exit_vertical_dead_zone_ratio=cls._config_float(
                config, "target_lock_exit_vertical_dead_zone_ratio", 0.10
            ),
        )

    def compute_command(
        self,
        target_result: Dict[str, object],
        frame_width: int,
        frame_height: int,
    ) -> RCCommand:
        """Convert target position and area into a safe RC command."""
        frame_area = max(1, int(frame_width) * int(frame_height))
        frame_center_x = int(frame_width) // 2
        frame_center_y = int(frame_height) // 2
        if not target_result or not target_result.get("found"):
            self._clear_lock_state()
            self.last_debug = FollowDebugInfo(
                frame_center_x=frame_center_x,
                frame_center_y=frame_center_y,
                target_state="LOST",
            )
            return self.hover()

        center = target_result.get("center")
        if center is None:
            target_center_x = target_result.get("target_center_x")
            target_center_y = target_result.get("target_center_y")
            if target_center_x is not None and target_center_y is not None:
                center = (target_center_x, target_center_y)
        area = float(target_result.get("area") or 0.0)
        if center is None:
            self._clear_lock_state()
            self.last_debug = FollowDebugInfo(
                frame_center_x=frame_center_x,
                frame_center_y=frame_center_y,
                target_area=area,
                area_ratio=area / frame_area,
                target_state="LOST",
            )
            return self.hover()

        target_x, target_y = center  # type: ignore[misc]
        horizontal_error = int(target_x) - frame_center_x
        horizontal_error_ratio = horizontal_error / max(1.0, frame_width / 2.0)
        vertical_error = int(target_y) - frame_center_y
        vertical_error_ratio = vertical_error / max(1.0, frame_height / 2.0)
        area_ratio = area / frame_area

        left_right = 0
        yaw = self._compute_yaw(horizontal_error_ratio)
        horizontal_centered = abs(horizontal_error_ratio) <= self.horizontal_dead_zone_ratio
        vertical_centered = abs(vertical_error_ratio) <= self.vertical_dead_zone_ratio
        detected_this_frame = not bool(target_result.get("is_predicted", False))

        target_state = "FOUND"
        if self._target_locked:
            if (
                not detected_this_frame
                or self._outside_lock_exit_range(horizontal_error_ratio, vertical_error_ratio, area_ratio)
            ):
                self._clear_lock_state()
            else:
                target_state = "LOCKED"

        if target_state == "LOCKED":
            up_down = 0
            yaw = 0
            forward_backward = 0
        else:
            is_stable = (
                detected_this_frame
                and horizontal_centered
                and vertical_centered
                and self.target_area_ratio_min <= area_ratio <= self.target_area_ratio_max
            )
            if is_stable:
                self._stable_frame_count += 1
                if self._stable_frame_count >= self.target_lock_stable_frames:
                    self._target_locked = True
                    target_state = "LOCKED"
            else:
                self._stable_frame_count = 0

            # 三轴可同时对准。存在转向或上下修正时，前后速度固定限为较低值，
            # 避免飞机在姿态尚未稳定时以前后最大速度移动。
            up_down = self._compute_vertical(vertical_error_ratio)
            forward_backward = self._compute_forward(area_ratio)
            if (yaw != 0 or up_down != 0) and forward_backward != 0:
                forward_backward = (
                    self.forward_speed_while_aligning
                    if forward_backward > 0
                    else -self.forward_speed_while_aligning
                )

            if target_state == "LOCKED":
                up_down = 0
                yaw = 0
                forward_backward = 0

        limited = self.safety_manager.limit_rc_command(
            left_right,
            forward_backward,
            up_down,
            yaw,
        )
        command = RCCommand(*limited)
        self.last_debug = FollowDebugInfo(
            target_center_x=int(target_x),
            target_center_y=int(target_y),
            frame_center_x=frame_center_x,
            frame_center_y=frame_center_y,
            horizontal_error=horizontal_error,
            horizontal_error_ratio=horizontal_error_ratio,
            vertical_error=vertical_error,
            vertical_error_ratio=vertical_error_ratio,
            target_area=area,
            area_ratio=area_ratio,
            target_state=target_state,
            yaw=command.yaw,
            left_right=command.left_right,
            forward_backward=command.forward_backward,
            up_down=command.up_down,
        )
        return command

    def reset(self) -> None:
        """Clear lock state before a new independent follow task."""
        self._clear_lock_state()
        self.last_debug = FollowDebugInfo()

    def hover(self) -> RCCommand:
        """Return a zero-velocity command."""
        limited = self.safety_manager.limit_rc_command(0, 0, 0, 0)
        return RCCommand(*limited)

    def _clear_lock_state(self) -> None:
        self._target_locked = False
        self._stable_frame_count = 0

    def _outside_lock_exit_range(
        self,
        horizontal_error_ratio: float,
        vertical_error_ratio: float,
        area_ratio: float,
    ) -> bool:
        """Return whether a locked target moved enough to resume following."""
        return (
            abs(horizontal_error_ratio) > self.target_lock_exit_horizontal_dead_zone_ratio
            or abs(vertical_error_ratio) > self.target_lock_exit_vertical_dead_zone_ratio
            or area_ratio < self.target_lock_exit_area_ratio_min
            or area_ratio > self.target_lock_exit_area_ratio_max
        )

    def _compute_yaw(self, horizontal_error_ratio: float) -> int:
        """Compute yaw-first horizontal tracking.

        Tello/djitellopy convention: positive yaw rotates right clockwise,
        negative yaw rotates left. Since error is target_x - frame_center_x,
        a left-side target produces negative yaw and a right-side target
        produces positive yaw.
        """
        if abs(horizontal_error_ratio) <= self.horizontal_dead_zone_ratio:
            return 0

        raw_yaw = int(horizontal_error_ratio * self.yaw_kp)
        yaw = self._apply_minimum_speed(raw_yaw, self.minimum_yaw_speed)
        return self._clamp_int(yaw, -self.maximum_yaw_speed, self.maximum_yaw_speed)

    def _compute_vertical(self, vertical_error_ratio: float) -> int:
        """Compute up/down tracking from target vertical image position.

        Tello RC convention: positive up_down rises, negative up_down descends.
        OpenCV y grows downward, so a target above the center has a negative
        error and requires positive up_down to climb.
        """
        if abs(vertical_error_ratio) <= self.vertical_dead_zone_ratio:
            return 0
        if vertical_error_ratio < 0:
            return self.vertical_speed
        return -self.vertical_speed

    def _compute_forward(self, area_ratio: float) -> int:
        """Use full forward/backward speed outside the hover distance band."""
        if area_ratio < self.target_area_ratio_min:
            return self.maximum_forward_speed

        if area_ratio > self.target_area_ratio_max:
            return -self.maximum_forward_speed

        return 0

    @staticmethod
    def _apply_minimum_speed(value: int, minimum_speed: int) -> int:
        if value == 0:
            return 0
        if abs(value) < minimum_speed:
            return minimum_speed if value > 0 else -minimum_speed
        return value

    @staticmethod
    def _clamp_int(value: int, lower: int, upper: int) -> int:
        return max(lower, min(upper, int(value)))

    @staticmethod
    def _clamp_float(value: float, lower: float, upper: float, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        return max(lower, min(upper, numeric))

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
    def _positive_float(value: float, default: float) -> float:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = default
        return max(0.0001, numeric)

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
