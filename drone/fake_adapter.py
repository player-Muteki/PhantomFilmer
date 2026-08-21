"""Fake drone adapter for no-hardware validation."""

from math import sin
from time import monotonic
from typing import Any

import numpy as np

from .drone_adapter import DroneAdapter


class FakeDroneAdapter(DroneAdapter):
    """Simulated drone backend used when RoboMaster TT / Tello is unavailable."""

    def __init__(
        self,
        verbose_rc: bool = True,
        camera_width: int = 640,
        camera_height: int = 480,
        target_speed: int = 3,
        target_lost_interval_seconds: float = 12.0,
        target_lost_duration_seconds: float = 2.0,
        takeoff_height_cm: int = 70,
    ) -> None:
        self.connected = False
        self.streaming = False
        self.height_cm = 0
        self.battery_percent = 80
        self.verbose_rc = verbose_rc
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.target_speed = target_speed
        self.target_lost_interval_seconds = target_lost_interval_seconds
        self.target_lost_duration_seconds = target_lost_duration_seconds
        self.takeoff_height_cm = max(20, int(takeoff_height_cm))
        self.last_rc_command = (0, 0, 0, 0)
        self.frame_index = 0
        self.target_visible = True
        self.force_target_visible = None
        self._started_at = monotonic()
        self._last_rc_at = self._started_at
        self.yaw_degrees = 0.0
        self.front_distance_cm = None

    def connect(self) -> None:
        """Simulate drone connection."""
        self.connected = True
        print("模拟无人机连接成功")

    def takeoff(self) -> None:
        """Simulate takeoff at the configured base height."""
        self.height_cm = self.takeoff_height_cm
        print("模拟起飞")

    def land(self) -> None:
        """Simulate landing and reset height."""
        self.height_cm = 0
        print("模拟降落")

    def stop(self) -> None:
        """Simulate emergency stop."""
        print("模拟急停")

    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        """Save and optionally print simulated RC command values."""
        now = monotonic()
        elapsed = max(0.0, min(0.2, now - self._last_rc_at))
        self._last_rc_at = now
        self.yaw_degrees = ((self.yaw_degrees + int(yaw) * elapsed + 180) % 360) - 180
        self.last_rc_command = (
            int(left_right),
            int(forward_backward),
            int(up_down),
            int(yaw),
        )
        if not self.verbose_rc:
            return
        print(
            "模拟控制量："
            f"left_right={left_right}, "
            f"forward_backward={forward_backward}, "
            f"up_down={up_down}, "
            f"yaw={yaw}"
        )

    def get_battery(self) -> int:
        """Return simulated battery percentage."""
        return self.battery_percent

    def get_cached_battery(self) -> int:
        """Return the already-local simulated battery state."""
        return self.battery_percent

    def get_height(self) -> int:
        """Return simulated downward ground clearance in centimeters."""
        return self.height_cm

    def get_yaw(self) -> int:
        """Return simulated wrapped yaw in the same range as Tello telemetry."""
        return int(round(self.yaw_degrees))

    def get_front_distance_cm(self):
        """Return the configurable simulated front ToF reading."""
        return self.front_distance_cm

    def stream_on(self) -> None:
        """Simulate video stream startup."""
        self.streaming = True
        print("模拟视频流开启")

    def stream_off(self) -> None:
        """Simulate video stream shutdown."""
        self.streaming = False
        print("模拟视频流关闭")

    def get_frame(self) -> Any:
        """Return a dynamic BGR test image with a moving simulated subject."""
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-python 依赖：请先安装 requirements.txt。") from exc

        self.frame_index += 1
        frame = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            "FAKE CAMERA",
            (20, self.camera_height - 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (180, 180, 180),
            2,
        )
        cv2.putText(
            frame,
            f"battery={self.battery_percent}% height={self.height_cm}cm rc={self.last_rc_command}",
            (20, self.camera_height - 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (180, 180, 180),
            1,
        )

        self.target_visible = self._is_target_visible()
        if not self.target_visible:
            cv2.putText(
                frame,
                "target lost",
                (self.camera_width // 2 - 70, self.camera_height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (180, 180, 180),
                2,
            )
            return frame

        center_x = int(
            self.camera_width / 2
            + sin(self.frame_index * 0.035 * self.target_speed) * self.camera_width * 0.30
        )
        center_y = int(
            self.camera_height / 2
            + sin(self.frame_index * 0.023 * self.target_speed) * self.camera_height * 0.22
        )
        half_size = int(25 + abs(sin(self.frame_index * 0.025 * self.target_speed)) * 45)
        x1 = max(0, center_x - half_size)
        y1 = max(0, center_y - half_size)
        x2 = min(self.camera_width - 1, center_x + half_size)
        y2 = min(self.camera_height - 1, center_y + half_size)
        head_radius = max(6, half_size // 4)
        cv2.circle(frame, (center_x, y1 + head_radius), head_radius, (210, 210, 210), -1)
        body_top = min(y2, y1 + head_radius * 2)
        cv2.rectangle(frame, (x1, body_top), (x2, y2), (160, 160, 160), -1)
        return frame

    def _is_target_visible(self) -> bool:
        """Return whether the fake target should be drawn this frame."""
        if self.force_target_visible is not None:
            return bool(self.force_target_visible)

        lost_interval = max(0.1, float(self.target_lost_interval_seconds))
        lost_duration = max(0.0, float(self.target_lost_duration_seconds))
        cycle_seconds = lost_interval + lost_duration
        cycle_position = (monotonic() - self._started_at) % cycle_seconds
        return cycle_position < lost_interval
