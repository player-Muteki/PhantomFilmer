"""Real-aircraft connection and single-owner video services for the WebUI."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from threading import Condition, Event, RLock, Thread
from time import monotonic, sleep
from typing import Any, Callable, Iterator, Optional

from console.tools import ConsoleTools
from vision.camera import CameraStream


class ConnectionState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    VERIFIED = "VERIFIED"
    DEGRADED = "DEGRADED"
    CLOSING = "CLOSING"


@dataclass
class TelemetryCache:
    battery: int = 0
    height: int = 0
    yaw: Optional[int] = None
    front_distance: Optional[float] = None
    front_tof_supported: Optional[bool] = None
    last_success_at: Optional[float] = None
    last_error: Optional[str] = None


class ConnectionService:
    """Verify a real SDK session and publish cached, rate-limited telemetry."""

    def __init__(
        self,
        tools: ConsoleTools,
        health_interval_seconds: float = 2.0,
        failure_limit: int = 3,
        freshness_seconds: float = 5.0,
    ) -> None:
        self.tools = tools
        self.health_interval_seconds = max(0.5, float(health_interval_seconds))
        self.failure_limit = max(1, int(failure_limit))
        self.freshness_seconds = max(1.0, float(freshness_seconds))
        self.state = ConnectionState.DISCONNECTED
        self.cache = TelemetryCache()
        self._failures = 0
        self._lock = RLock()
        self._stop_event = Event()
        self._thread: Optional[Thread] = None
        self._degraded_callback: Optional[Callable[[], None]] = None

    def set_degraded_callback(self, callback: Callable[[], None]) -> None:
        self._degraded_callback = callback

    def connect(self) -> dict[str, Any]:
        with self._lock:
            if self.state == ConnectionState.VERIFIED and self.is_fresh():
                return self.snapshot()
            if self.state == ConnectionState.CONNECTING:
                raise RuntimeError("真机连接验证正在进行，请稍候。")
            self.state = ConnectionState.CONNECTING
            self.cache.last_error = None

        try:
            self.tools.connect()
            # A second, independent status probe is required after the adapter's
            # SDK command/battery handshake.  Failure closes the session.
            status = self.tools.get_status()
            self._validate_status(status)
            yaw = self._optional_read(self.tools._drone.get_yaw)
            front_distance, front_supported = self._read_optional_front_tof()
            with self._lock:
                self._publish_status(status, yaw, front_distance, front_supported)
                self._failures = 0
                self.state = ConnectionState.VERIFIED
            self._start_monitor()
            return self.snapshot()
        except Exception as exc:
            try:
                self.tools.close()
            except Exception:
                pass
            with self._lock:
                self.state = ConnectionState.DISCONNECTED
                self.cache.last_error = str(exc)
            if isinstance(exc, RuntimeError):
                raise RuntimeError(f"真机连接验证失败：{exc}") from exc
            raise RuntimeError(f"真机连接验证失败：{exc}") from exc

    def require_verified(self) -> None:
        with self._lock:
            if self.state != ConnectionState.VERIFIED:
                raise RuntimeError("真机连接尚未验证或已经断线，禁止启动飞行任务。")
            if not self.is_fresh():
                self.state = ConnectionState.DEGRADED
                raise RuntimeError("真机状态已经过期，请等待连接恢复后再启动任务。")

    def record_command_failure(self, error: Exception) -> None:
        callback = None
        with self._lock:
            self._failures += 1
            self.cache.last_error = str(error)
            if self._failures >= self.failure_limit:
                self.state = ConnectionState.DEGRADED
                callback = self._degraded_callback
        if callback is not None:
            callback()

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            age = None
            if self.cache.last_success_at is not None:
                age = max(0.0, monotonic() - self.cache.last_success_at)
            verified = self.state == ConnectionState.VERIFIED and (
                age is not None and age <= self.freshness_seconds
            )
            return {
                "battery": self.cache.battery,
                "height": self.cache.height,
                "yaw": self.cache.yaw,
                "front_distance": self.cache.front_distance,
                "front_tof_supported": self.cache.front_tof_supported,
                "connection_state": self.state.value,
                "connection_verified": verified,
                "status_age_seconds": age,
                "connection_error": self.cache.last_error,
            }

    def is_fresh(self) -> bool:
        if self.cache.last_success_at is None:
            return False
        return monotonic() - self.cache.last_success_at <= self.freshness_seconds

    def close(self) -> None:
        with self._lock:
            self.state = ConnectionState.CLOSING
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=self.health_interval_seconds + 1.0)
        with self._lock:
            self._thread = None
            self.state = ConnectionState.DISCONNECTED

    def _start_monitor(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = Thread(
                target=self._monitor_loop,
                name="PhantomFilmerWebHealth",
                daemon=True,
            )
            self._thread.start()

    def _monitor_loop(self) -> None:
        while not self._stop_event.wait(self.health_interval_seconds):
            with self._lock:
                if self.state not in (ConnectionState.VERIFIED, ConnectionState.DEGRADED):
                    continue
            try:
                status = self.tools.get_status()
                self._validate_status(status)
                yaw = self._optional_read(self.tools._drone.get_yaw)
                front_distance, front_supported = self._read_optional_front_tof()
                with self._lock:
                    self._publish_status(status, yaw, front_distance, front_supported)
                    self._failures = 0
                    self.state = ConnectionState.VERIFIED
            except Exception as exc:
                self.record_command_failure(exc)

    def _publish_status(
        self,
        status: dict[str, Any],
        yaw: Optional[Any],
        front_distance: Optional[float],
        front_supported: bool,
    ) -> None:
        self.cache.battery = int(status["battery"])
        self.cache.height = int(status["height"])
        self.cache.yaw = None if yaw is None else int(yaw)
        self.cache.front_distance = front_distance
        self.cache.front_tof_supported = front_supported
        self.cache.last_success_at = monotonic()
        self.cache.last_error = None

    @staticmethod
    def _validate_status(status: dict[str, Any]) -> None:
        battery = int(status["battery"])
        height = int(status["height"])
        if not 0 <= battery <= 100:
            raise RuntimeError(f"电量响应超出有效范围：{battery}")
        if not 0 <= height <= 1000:
            raise RuntimeError(f"离地高度响应超出有效范围：{height}")

    @staticmethod
    def _optional_read(reader: Callable[[], Any]) -> Optional[Any]:
        try:
            return reader()
        except (RuntimeError, NotImplementedError, AttributeError):
            return None

    def _read_optional_front_tof(self) -> tuple[Optional[float], bool]:
        try:
            value = self.tools._drone.get_front_distance_cm()
            return (None if value is None else float(value), True)
        except (RuntimeError, NotImplementedError, AttributeError):
            # Expansion ToF is optional and never contributes to connection
            # failure counts or the Web flight-start gate.
            return None, False


class VideoOwner(str, Enum):
    NONE = "NONE"
    WEB_PREVIEW = "WEB_PREVIEW"
    FOLLOW_SESSION = "FOLLOW_SESSION"


class VideoHub:
    """One camera producer shared by every browser MJPEG consumer."""

    def __init__(self, tools: ConsoleTools, jpeg_quality: int = 80) -> None:
        self.tools = tools
        self.jpeg_quality = max(40, min(95, int(jpeg_quality)))
        self.owner = VideoOwner.NONE
        self._condition = Condition(RLock())
        self._preview_stop = Event()
        self._preview_thread: Optional[Thread] = None
        self._camera: Optional[CameraStream] = None
        self._frame: Optional[Any] = None
        self._sequence = 0
        self._closed = False
        self.last_error: Optional[str] = None

    @property
    def active(self) -> bool:
        with self._condition:
            return self.owner != VideoOwner.NONE

    def start_preview(self) -> None:
        with self._condition:
            if self._closed:
                raise RuntimeError("视频服务已经关闭。")
            if self.owner == VideoOwner.FOLLOW_SESSION:
                return
            if self._preview_thread is not None and self._preview_thread.is_alive():
                return
            camera = CameraStream(
                self.tools._drone,
                width=self.tools.frame_width,
                height=self.tools.frame_height,
            )
            camera.start()
            self._camera = camera
            self._preview_stop.clear()
            self.owner = VideoOwner.WEB_PREVIEW
            self.last_error = None
            self._preview_thread = Thread(
                target=self._preview_loop,
                args=(camera,),
                name="PhantomFilmerWebVideo",
                daemon=True,
            )
            self._preview_thread.start()

    def handoff_to_task(self) -> None:
        self.stop_preview()
        with self._condition:
            self.owner = VideoOwner.FOLLOW_SESSION
            self.last_error = None
            self._condition.notify_all()

    def publish_task_frame(self, frame: Any) -> None:
        with self._condition:
            if self.owner != VideoOwner.FOLLOW_SESSION or self._closed:
                return
            self._publish_frame(frame)

    def task_finished(self) -> None:
        with self._condition:
            if self.owner == VideoOwner.FOLLOW_SESSION:
                self.owner = VideoOwner.NONE
            self._condition.notify_all()

    def stop_preview(self) -> None:
        with self._condition:
            thread = self._preview_thread
            self._preview_stop.set()
        if thread is not None and thread.is_alive():
            thread.join(timeout=3.0)
        with self._condition:
            if self.owner == VideoOwner.WEB_PREVIEW:
                self.owner = VideoOwner.NONE
            self._preview_thread = None
            self._camera = None
            self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            self._closed = True
        self.stop_preview()
        with self._condition:
            self.owner = VideoOwner.NONE
            self._frame = None
            self._condition.notify_all()

    def iter_mjpeg(self) -> Iterator[bytes]:
        import cv2

        with self._condition:
            last_sequence = self._sequence
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._closed or self._sequence != last_sequence,
                    timeout=2.0,
                )
                if self._closed:
                    return
                if self._sequence == last_sequence or self._frame is None:
                    continue
                frame = self._frame.copy()
                last_sequence = self._sequence
            ok, encoded = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
            )
            if ok:
                yield (
                    b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                    + encoded.tobytes()
                    + b"\r\n"
                )

    def _preview_loop(self, camera: CameraStream) -> None:
        try:
            while not self._preview_stop.is_set():
                frame = camera.read_frame()
                if frame is None:
                    sleep(0.02)
                    continue
                with self._condition:
                    self._publish_frame(frame)
        except Exception as exc:
            with self._condition:
                self.last_error = str(exc)
        finally:
            try:
                camera.stop()
            except RuntimeError:
                pass
            with self._condition:
                if self.owner == VideoOwner.WEB_PREVIEW:
                    self.owner = VideoOwner.NONE
                self._condition.notify_all()

    def _publish_frame(self, frame: Any) -> None:
        self._frame = frame.copy()
        self._sequence += 1
        self._condition.notify_all()
