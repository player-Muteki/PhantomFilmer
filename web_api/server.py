"""Serve real-device telemetry and embedded MJPEG video to the local WebUI.

The service binds to loopback only. It never creates an OpenCV window and it
does not connect to an aircraft until the user requests POST /api/drone/connect.
"""

from __future__ import annotations

import json
import signal
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import RLock
from time import monotonic, sleep
from typing import Any, Callable, Iterator, Optional

from .tello_adapter import RealTelloAdapter


HOST = "127.0.0.1"
PORT = 8765
VIDEO_BOUNDARY = "frame"


class DroneWebService:
    """Own one real-aircraft session shared by all local HTTP requests."""

    def __init__(self, adapter_factory: Callable[[], Any] = RealTelloAdapter) -> None:
        self._adapter_factory = adapter_factory
        self._adapter: Optional[Any] = None
        self._lock = RLock()
        self._video_ready = False
        self._last_frame_at: Optional[float] = None
        self._control_hz = 0.0
        self._battery: Optional[int] = None
        self._front_tof: Optional[float] = None
        self._front_tof_checked_at = 0.0

    @property
    def connected(self) -> bool:
        adapter = self._adapter
        return bool(adapter is not None and getattr(adapter, "connected", False))

    def connect(self) -> dict[str, Any]:
        """Connect to a real aircraft, then and only then request its stream."""
        with self._lock:
            if self.connected:
                return self.status(probe_video=True)

            adapter = self._adapter_factory()
            self._adapter = adapter
            self._video_ready = False
            self._control_hz = 0.0
            self._last_frame_at = None
            try:
                adapter.connect()
                self._battery = getattr(adapter, "last_connection_battery", None)
                if self._battery is None:
                    self._battery = adapter.get_battery()
                adapter.stream_on()
                self._probe_video(timeout_seconds=5.0)
                return self.status(probe_video=False)
            except Exception:
                try:
                    adapter.stop()
                finally:
                    self._adapter = None
                    self._video_ready = False
                raise

    def status(self, probe_video: bool = True) -> dict[str, Any]:
        """Return fresh real-device telemetry without manufacturing fallback data."""
        with self._lock:
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接。")

            adapter = self._adapter
            if probe_video and not self._video_ready:
                self._probe_video(timeout_seconds=0.2)

            try:
                self._battery = adapter.get_cached_battery()
            except RuntimeError:
                pass

            try:
                height = adapter.get_height()
            except RuntimeError:
                height = None

            now = monotonic()
            if now - self._front_tof_checked_at >= 2.0:
                self._front_tof_checked_at = now
                try:
                    self._front_tof = adapter.get_front_distance_cm()
                except RuntimeError:
                    self._front_tof = None

            return {
                "battery": self._battery,
                "heightCm": height,
                "frontTofCm": self._front_tof,
                "controlHz": round(self._control_hz, 1),
                "flightState": "地面待机",
                "targetConfirmed": False,
                "phase": "连接",
                "videoReady": self._video_ready,
            }

    def stop(self) -> None:
        """Clear RC output, stop the stream, and close this ground session."""
        with self._lock:
            adapter = self._adapter
            self._adapter = None
            self._video_ready = False
            self._last_frame_at = None
            self._control_hz = 0.0
            self._front_tof = None
            self._front_tof_checked_at = 0.0
            if adapter is not None:
                adapter.stop()

    def emergency_land(self) -> None:
        """Request landing, then close the connection regardless of outcome."""
        with self._lock:
            adapter = self._adapter
            if adapter is None or not getattr(adapter, "connected", False):
                raise RuntimeError("真机未连接，无法发送降落指令。")
            try:
                adapter.move_rc(0, 0, 0, 0)
                adapter.land()
            finally:
                self._adapter = None
                self._video_ready = False
                self._last_frame_at = None
                self._control_hz = 0.0
                self._front_tof = None
                self._front_tof_checked_at = 0.0
                adapter.stop()

    def mjpeg_frames(self) -> Iterator[bytes]:
        """Yield JPEG frames for one browser client without any GUI window."""
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-python，无法编码网页视频流。") from exc

        while self.connected:
            adapter = self._adapter
            if adapter is None:
                break
            try:
                frame = adapter.get_frame()
                if not self._is_real_frame(frame):
                    sleep(0.05)
                    continue
                ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if not ok:
                    sleep(0.05)
                    continue
                self._mark_frame()
                payload = encoded.tobytes()
                yield (
                    f"--{VIDEO_BOUNDARY}\r\n".encode()
                    + b"Content-Type: image/jpeg\r\n"
                    + f"Content-Length: {len(payload)}\r\n\r\n".encode()
                    + payload
                    + b"\r\n"
                )
                sleep(0.03)
            except RuntimeError:
                if not self.connected:
                    break
                sleep(0.1)

    def _probe_video(self, timeout_seconds: float) -> None:
        adapter = self._adapter
        if adapter is None or not self.connected:
            return
        deadline = monotonic() + timeout_seconds
        while monotonic() <= deadline:
            try:
                frame = adapter.get_frame()
                if self._is_real_frame(frame):
                    self._video_ready = True
                    self._mark_frame()
                    return
            except RuntimeError:
                pass
            sleep(0.05)

    @staticmethod
    def _is_real_frame(frame: Any) -> bool:
        """Reject djitellopy's all-zero placeholder before the first UDP frame."""
        if frame is None or getattr(frame, "size", 0) == 0:
            return False
        any_pixel = getattr(frame, "any", None)
        return bool(any_pixel()) if callable(any_pixel) else True

    def _mark_frame(self) -> None:
        now = monotonic()
        if self._last_frame_at is not None:
            interval = now - self._last_frame_at
            if interval > 0:
                instant_hz = min(60.0, 1.0 / interval)
                self._control_hz = (
                    instant_hz if self._control_hz == 0 else self._control_hz * 0.8 + instant_hz * 0.2
                )
        self._last_frame_at = now


SERVICE = DroneWebService()


class DroneRequestHandler(BaseHTTPRequestHandler):
    """Minimal loopback-only API used by the Next.js reverse proxy."""

    server_version = "PhantomFilmerWeb/1.0"

    def do_GET(self) -> None:  # noqa: N802
        if self.path == "/api/health":
            self._json(HTTPStatus.OK, {"ok": True, "connected": SERVICE.connected})
            return
        if self.path == "/api/drone/status":
            self._run_json(SERVICE.status)
            return
        if self.path == "/api/drone/video/stream":
            self._video_stream()
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在。"})

    def do_POST(self) -> None:  # noqa: N802
        if self.path == "/api/drone/connect":
            self._run_json(SERVICE.connect)
            return
        if self.path == "/api/drone/stop":
            self._run_empty(SERVICE.stop)
            return
        if self.path == "/api/drone/emergency-land":
            self._run_empty(SERVICE.emergency_land)
            return
        self._json(HTTPStatus.NOT_FOUND, {"error": "接口不存在。"})

    def log_message(self, format_string: str, *args: Any) -> None:
        print(f"[WebAPI] {self.address_string()} - {format_string % args}")

    def _run_json(self, action: Callable[[], dict[str, Any]]) -> None:
        try:
            self._json(HTTPStatus.OK, action())
        except Exception as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})

    def _run_empty(self, action: Callable[[], None]) -> None:
        try:
            action()
            self._json(HTTPStatus.OK, {"ok": True})
        except Exception as exc:
            self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})

    def _video_stream(self) -> None:
        if not SERVICE.connected:
            self._json(HTTPStatus.CONFLICT, {"error": "真机未连接，视频流未开启。"})
            return
        self.send_response(HTTPStatus.OK)
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={VIDEO_BOUNDARY}")
        self.end_headers()
        try:
            for frame in SERVICE.mjpeg_frames():
                self.wfile.write(frame)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _json(self, status: HTTPStatus, body: dict[str, Any]) -> None:
        payload = json.dumps(body, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(payload)


def main() -> int:
    """Run the local bridge until interrupted, always releasing the aircraft."""
    server = ThreadingHTTPServer((HOST, PORT), DroneRequestHandler)

    def shutdown(_signum: int, _frame: Any) -> None:
        SERVICE.stop()
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)
    print(f"PhantomFilmer 真机服务：http://{HOST}:{PORT}")
    print("服务仅监听本机；点击 WebUI 的“连接真机”后才会连接无人机并开启视频。")
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        SERVICE.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
