"""Serve real-device telemetry and embedded MJPEG video to the local WebUI.

The service binds to loopback only. It never creates an OpenCV window and it
does not connect to an aircraft until the user requests POST /api/drone/connect.
"""

from __future__ import annotations

import json
import signal
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from threading import Event, RLock, Thread
from time import monotonic, sleep
from typing import Any, Callable, Iterator, Optional

from .tello_adapter import RealTelloAdapter


HOST = "127.0.0.1"
PORT = 8765
VIDEO_BOUNDARY = "frame"
MIN_TAKEOFF_BATTERY = 20
MAX_RC_SPEED = 35
RC_WATCHDOG_SECONDS = 0.4
FRONT_STOP_DISTANCE_CM = 60.0
MIN_DESCENT_HEIGHT_CM = 40
MAX_ASCENT_HEIGHT_CM = 200


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
        self._front_tof_state = "unavailable"
        self._height: Optional[int] = None
        self._airborne = False
        self._flight_state = "未连接"
        self._phase = "连接"
        self._last_rc_at = 0.0
        self._rc_active = False
        self._watchdog_stop = Event()
        self._watchdog = Thread(target=self._watch_rc, daemon=True)
        self._tof_monitor = Thread(target=self._monitor_front_tof, daemon=True)
        self._watchdog.start()
        self._tof_monitor.start()

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
            self._airborne = False
            self._flight_state = "正在连接"
            self._phase = "连接"
            try:
                adapter.connect()
                self._battery = getattr(adapter, "last_connection_battery", None)
                if self._battery is None:
                    self._battery = adapter.get_battery()
                adapter.stream_on()
                self._probe_video(timeout_seconds=5.0)
                self._refresh_front_tof(force=True)
                self._flight_state = "地面待机"
                self._phase = "检查"
                return self.status(probe_video=False)
            except Exception:
                try:
                    adapter.stop()
                finally:
                    self._adapter = None
                    self._video_ready = False
                    self._flight_state = "连接失败"
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
                self._height = adapter.get_height()
            except RuntimeError:
                self._height = None

            front_tof_ready = self._front_tof_state in ("clear", "out_of_range")
            can_takeoff = bool(
                not self._airborne
                and self._video_ready
                and self._battery is not None
                and self._battery >= MIN_TAKEOFF_BATTERY
                and self._height is not None
                and front_tof_ready
            )

            return {
                "battery": self._battery,
                "heightCm": self._height,
                "frontTofCm": self._front_tof,
                "frontTofState": self._front_tof_state,
                "controlHz": round(self._control_hz, 1),
                "flightState": self._flight_state,
                "targetConfirmed": False,
                "phase": self._phase,
                "videoReady": self._video_ready,
                "airborne": self._airborne,
                "canTakeoff": can_takeoff,
                "rcEnabled": self._airborne,
                "preflight": {
                    "sdk": True,
                    "video": self._video_ready,
                    "battery": self._battery is not None and self._battery >= MIN_TAKEOFF_BATTERY,
                    "bottomTof": self._height is not None,
                    "frontTof": front_tof_ready,
                },
            }

    def takeoff(self) -> dict[str, Any]:
        """Take off only after all locally verifiable preflight gates pass."""
        with self._lock:
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接，不能起飞。")
            if self._airborne:
                raise RuntimeError("真机已经处于空中状态。")
            current = self.status(probe_video=True)
            if not current["canTakeoff"]:
                raise RuntimeError("起飞检查未通过：请确认视频、电量和 ToF 传感器均正常。")
            self._flight_state = "正在起飞"
            self._phase = "起飞"
            try:
                authorize_takeoff = getattr(
                    self._adapter, "authorize_next_takeoff", None
                )
                if callable(authorize_takeoff):
                    authorize_takeoff()
                self._adapter.takeoff()
            except Exception:
                self._flight_state = "起飞失败"
                self._phase = "检查"
                raise
            self._airborne = True
            self._flight_state = "手动悬停"
            self._phase = "手动飞行"
            self._send_hover()
            return self.status(probe_video=False)

    def land(self) -> dict[str, Any]:
        """Clear movement and land while retaining telemetry and video."""
        with self._lock:
            if not self.connected or self._adapter is None:
                raise RuntimeError("真机未连接，无法降落。")
            if not self._airborne:
                raise RuntimeError("真机当前不在空中。")
            self._flight_state = "正在降落"
            self._phase = "降落"
            self._send_hover()
            self._adapter.land()
            self._airborne = False
            self._flight_state = "地面待机"
            self._phase = "检查"
            return self.status(probe_video=False)

    def hover(self) -> dict[str, Any]:
        """Immediately clear all four RC channels."""
        with self._lock:
            self._require_airborne()
            self._send_hover()
            self._flight_state = "手动悬停"
            return self.status(probe_video=False)

    def move_rc(self, command: dict[str, Any]) -> dict[str, Any]:
        """Apply one bounded manual RC command; a backend watchdog clears it."""
        with self._lock:
            self._require_airborne()
            channels = {
                key: self._bounded_channel(command.get(key, 0))
                for key in ("leftRight", "forwardBack", "upDown", "yaw")
            }
            forward = channels["forwardBack"]
            vertical = channels["upDown"]
            if forward > 0:
                age = monotonic() - self._front_tof_checked_at
                if age > 1.2 or self._front_tof_state == "unavailable":
                    raise RuntimeError("前向 ToF 数据无效或过期，已禁止前进。")
                if self._front_tof is not None and self._front_tof <= FRONT_STOP_DISTANCE_CM:
                    raise RuntimeError("前方 60 cm 内存在障碍，已禁止前进。")
            if vertical < 0 and self._height is not None and self._height <= MIN_DESCENT_HEIGHT_CM:
                raise RuntimeError("当前高度过低，已禁止继续下降。")
            if vertical > 0 and self._height is not None and self._height >= MAX_ASCENT_HEIGHT_CM:
                raise RuntimeError("当前高度达到手动上升上限。")

            self._adapter.move_rc(
                channels["leftRight"],
                channels["forwardBack"],
                channels["upDown"],
                channels["yaw"],
            )
            self._last_rc_at = monotonic()
            self._rc_active = any(channels.values())
            self._flight_state = "手动飞行" if self._rc_active else "手动悬停"
            return {"ok": True, "flightState": self._flight_state}

    def stop(self) -> None:
        """Land first when necessary, then stop the stream and connection."""
        with self._lock:
            adapter = self._adapter
            if adapter is not None:
                if self._airborne:
                    self._send_hover()
                    adapter.land()
                adapter.stop()
            self._clear_session()

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
                self._clear_session()
                adapter.stop()

    def shutdown(self) -> None:
        """Release the aircraft and stop the local RC watchdog."""
        self.stop()
        self._watchdog_stop.set()

    def _refresh_front_tof(self, force: bool = False) -> None:
        adapter = self._adapter
        if adapter is None:
            self._front_tof = None
            self._front_tof_state = "unavailable"
            return
        now = monotonic()
        if not force and now - self._front_tof_checked_at < 2.0:
            return
        self._front_tof_checked_at = now
        try:
            distance = adapter.get_front_distance_cm()
            self._front_tof = distance
            self._front_tof_state = "out_of_range" if distance is None else (
                "blocked" if distance <= FRONT_STOP_DISTANCE_CM else "clear"
            )
        except RuntimeError:
            self._front_tof = None
            self._front_tof_state = "unavailable"

    def _require_airborne(self) -> None:
        if not self.connected or self._adapter is None:
            raise RuntimeError("真机未连接。")
        if not self._airborne:
            raise RuntimeError("真机尚未起飞，手动控制不可用。")

    @staticmethod
    def _bounded_channel(value: Any) -> int:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise RuntimeError("手动控制量格式无效。")
        return max(-MAX_RC_SPEED, min(MAX_RC_SPEED, int(value)))

    def _send_hover(self) -> None:
        adapter = self._adapter
        if adapter is None:
            return
        adapter.move_rc(0, 0, 0, 0)
        self._last_rc_at = monotonic()
        self._rc_active = False

    def _watch_rc(self) -> None:
        while not self._watchdog_stop.wait(0.1):
            with self._lock:
                if (
                    self._airborne
                    and self._rc_active
                    and monotonic() - self._last_rc_at > RC_WATCHDOG_SECONDS
                ):
                    try:
                        self._send_hover()
                        self._flight_state = "手动悬停"
                    except RuntimeError:
                        self._rc_active = False

    def _monitor_front_tof(self) -> None:
        """Poll the blocking expansion command without blocking manual RC handling."""
        while not self._watchdog_stop.wait(0.2):
            with self._lock:
                adapter = self._adapter if self.connected else None
            if adapter is None:
                continue
            try:
                distance = adapter.get_front_distance_cm()
                state = "out_of_range" if distance is None else (
                    "blocked" if distance <= FRONT_STOP_DISTANCE_CM else "clear"
                )
            except RuntimeError:
                distance = None
                state = "unavailable"
            with self._lock:
                if self._adapter is adapter:
                    self._front_tof = distance
                    self._front_tof_state = state
                    self._front_tof_checked_at = monotonic()

    def _clear_session(self) -> None:
        self._adapter = None
        self._video_ready = False
        self._last_frame_at = None
        self._control_hz = 0.0
        self._front_tof = None
        self._front_tof_checked_at = 0.0
        self._front_tof_state = "unavailable"
        self._height = None
        self._airborne = False
        self._flight_state = "未连接"
        self._phase = "连接"
        self._last_rc_at = 0.0
        self._rc_active = False

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
        if self.path == "/api/drone/takeoff":
            self._run_json(SERVICE.takeoff)
            return
        if self.path == "/api/drone/land":
            self._run_json(SERVICE.land)
            return
        if self.path == "/api/drone/hover":
            self._run_json(SERVICE.hover)
            return
        if self.path == "/api/drone/rc":
            try:
                command = self._read_json()
                self._json(HTTPStatus.OK, SERVICE.move_rc(command))
            except Exception as exc:
                self._json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": str(exc)})
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

    def _read_json(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length <= 0 or length > 4096:
            raise RuntimeError("请求内容为空或过大。")
        try:
            body = json.loads(self.rfile.read(length).decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("请求 JSON 无效。") from exc
        if not isinstance(body, dict):
            raise RuntimeError("请求必须是 JSON 对象。")
        return body

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
        SERVICE.shutdown()
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
        SERVICE.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
