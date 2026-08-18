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
        detector_type: str = "red",
        aruco_dictionary: str = "DICT_4X4_50",
        target_marker_id: int = 23,
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
        self.detector_type = str(detector_type).strip().lower()
        self.aruco_dictionary = str(aruco_dictionary)
        self.target_marker_id = int(target_marker_id)
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
        """Return a dynamic BGR test image with a moving red target."""
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。") from exc

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
        if self.detector_type == "aruco":
            self._draw_aruco_target(frame, cv2, x1, y1, x2, y2)
        else:
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), -1)
        return frame

    def _draw_aruco_target(
        self,
        frame: Any,
        cv2: Any,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> None:
        """Draw a detectable ArUco marker with a white quiet zone."""
        if not hasattr(cv2, "aruco"):
            raise RuntimeError(
                "当前 OpenCV 不包含 ArUco，请安装 requirements.txt 中的 "
                "opencv-contrib-python。"
            )

        marker_size = max(24, min(x2 - x1, y2 - y1))
        marker_x2 = x1 + marker_size
        marker_y2 = y1 + marker_size
        quiet_zone = max(6, marker_size // 8)
        cv2.rectangle(
            frame,
            (max(0, x1 - quiet_zone), max(0, y1 - quiet_zone)),
            (
                min(self.camera_width - 1, marker_x2 + quiet_zone),
                min(self.camera_height - 1, marker_y2 + quiet_zone),
            ),
            (255, 255, 255),
            -1,
        )

        dictionary_id = getattr(
            cv2.aruco,
            self.aruco_dictionary,
            cv2.aruco.DICT_4X4_50,
        )
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, "generateImageMarker"):
            marker = cv2.aruco.generateImageMarker(
                dictionary,
                self.target_marker_id,
                marker_size,
            )
        else:
            marker = np.zeros((marker_size, marker_size), dtype=np.uint8)
            cv2.aruco.drawMarker(
                dictionary,
                self.target_marker_id,
                marker_size,
                marker,
                1,
            )
        frame[y1:marker_y2, x1:marker_x2] = cv2.cvtColor(marker, cv2.COLOR_GRAY2BGR)

    def _is_target_visible(self) -> bool:
        """Return whether the fake target should be drawn this frame."""
        if self.force_target_visible is not None:
            return bool(self.force_target_visible)

        lost_interval = max(0.1, float(self.target_lost_interval_seconds))
        lost_duration = max(0.0, float(self.target_lost_duration_seconds))
        cycle_seconds = lost_interval + lost_duration
        cycle_position = (monotonic() - self._started_at) % cycle_seconds
        return cycle_position < lost_interval
