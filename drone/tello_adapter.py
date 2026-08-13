"""RoboMaster TT / Tello Talent adapter based on djitellopy.

This is the only module that imports djitellopy directly. The rest of the
project should use DroneAdapter so hardware details stay isolated here.
"""

import re
from threading import Lock
from typing import Any, Optional

from .drone_adapter import DroneAdapter


class TelloDroneAdapter(DroneAdapter):
    """Control RoboMaster TT / Tello Talent through djitellopy.Tello."""

    SDK_PORT = 8889
    # djitellopy runtime-checks this argument and accepts ``int`` only.
    COMMAND_TIMEOUT_SECONDS = 5

    def __init__(self, host: str = "192.168.10.1") -> None:
        self.host = host
        self._tello: Optional[Any] = None
        self.connected = False
        self.streaming = False
        self._takeoff_authorized = False
        self._command_lock = Lock()
        self.last_connection_battery: Optional[int] = None

    def authorize_next_takeoff(self) -> None:
        """Skip the adapter prompt once after an outer workflow confirms safety."""
        self._takeoff_authorized = True

    def revoke_takeoff_authorization(self) -> None:
        """Discard an unused outer-workflow takeoff authorization."""
        self._takeoff_authorized = False

    def connect(self) -> None:
        """Enter SDK mode after receiving the ``command`` acknowledgement."""
        if self.connected:
            return
        print(f"[Tello] Connecting to {self.host}:{self.SDK_PORT}")
        try:
            self._tello = self._create_tello()
            print("[Tello] Sending SDK command: command")
            print("[Tello] Waiting for response...")
            response = self._send_command_for_response(
                "command", timeout=self.COMMAND_TIMEOUT_SECONDS
            )
            self._require_ok_response(response)
            print("[Tello] Received: ok")
            self.connected = True
            print("[Tello] SDK connection established")
        except Exception as exc:
            self._tello = None
            self.connected = False
            if isinstance(exc, RuntimeError) and str(exc).startswith("[Tello]"):
                raise
            raise RuntimeError(f"[Tello] socket error: {exc}") from exc

        self.last_connection_battery = None
        try:
            battery = self._query_integer("battery?")
            if not 0 <= battery <= 100:
                raise RuntimeError(f"unexpected response: {battery}")
            self.last_connection_battery = battery
            print(f"[Tello] Connection verified, battery={battery}%")
        except Exception as exc:
            print(f"[Tello] WARNING: SDK connected, but battery verification failed: {exc}")

    def takeoff(self) -> None:
        """Take off only after the user explicitly confirms the action."""
        self._require_connection()
        if self._takeoff_authorized:
            self._takeoff_authorized = False
        else:
            answer = input("即将起飞，请确认周围安全并输入 YES 继续：").strip()
            if answer != "YES":
                raise RuntimeError("已取消起飞：未收到用户确认。")
        try:
            with self._command_lock:
                self._tello.takeoff()
        except Exception as exc:
            raise RuntimeError("起飞失败：请检查电量、桨叶保护罩和飞行环境。") from exc

    def land(self) -> None:
        """Land the aircraft if connected."""
        if not self.connected or self._tello is None:
            print("降落命令未执行：无人机尚未连接。")
            return
        try:
            with self._command_lock:
                self._tello.land()
        except Exception as exc:
            raise RuntimeError("降落失败：请保持无人机 Wi-Fi 连接并手动观察飞行状态。") from exc

    def stop(self) -> None:
        """Stop motion, disable stream, and release local state."""
        if self._tello is None:
            self.connected = False
            self.streaming = False
            return
        try:
            if self.connected:
                self._tello.send_rc_control(0, 0, 0, 0)
            if self.streaming:
                with self._command_lock:
                    self._tello.streamoff()
        except Exception as exc:
            print(f"停止无人机时出现异常：{exc}")
        finally:
            self.connected = False
            self.streaming = False
            self._takeoff_authorized = False

    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        """Send remote-control velocity values to the aircraft."""
        self._require_connection()
        try:
            self._tello.send_rc_control(left_right, forward_backward, up_down, yaw)
        except Exception as exc:
            raise RuntimeError("发送遥控指令失败：请检查 Wi-Fi 连接和无人机状态。") from exc

    def get_battery(self) -> int:
        """Return battery percentage from the aircraft."""
        self._require_connection()
        try:
            battery = self._query_integer("battery?")
            if not 0 <= battery <= 100:
                raise ValueError(f"电量超出范围：{battery}")
            return battery
        except Exception as exc:
            raise RuntimeError("读取电量失败：请确认无人机连接正常。") from exc

    def get_height(self) -> int:
        """Return downward TOF ground clearance in centimeters."""
        self._require_connection()
        try:
            return int(self._tello.get_distance_tof())
        except Exception as exc:
            raise RuntimeError("读取 TOF 离地高度失败：请确认无人机连接正常。") from exc

    def get_estimated_height(self) -> int:
        """Return the flight controller's raw h field for diagnostics only."""
        self._require_connection()
        try:
            return int(self._tello.get_height())
        except Exception as exc:
            raise RuntimeError("读取飞控估算高度失败：请确认无人机连接正常。") from exc

    def get_front_distance_cm(self) -> Optional[float]:
        """Read the top expansion ToF using SDK 3.0 ``EXT tof?`` (millimetres)."""
        self._require_connection()
        try:
            response = self._send_command_for_response("EXT tof?", timeout=1)
        except Exception as exc:
            raise RuntimeError("读取顶部前向 ToF 距离失败。") from exc
        if self._is_timeout_response(response):
            raise RuntimeError("读取顶部前向 ToF 距离失败：UDP timeout。")
        match = re.search(r"(?:tof\s+)?(\d+)", response.strip().lower())
        if match is None:
            raise RuntimeError(f"顶部前向 ToF 返回格式无效：{response!r}")
        millimetres = int(match.group(1))
        # The SDK documents 8192 as out-of-range, while the TT module observed
        # in the field also reports the adjacent sentinel values 8190/8191.
        if 8190 <= millimetres <= 8192:
            return None
        if not 0 < millimetres <= 1200:
            raise RuntimeError(f"顶部前向 ToF 返回范围无效：{millimetres} mm")
        return millimetres / 10.0

    def get_yaw(self) -> int:
        """Return the cached flight-controller yaw angle in degrees."""
        self._require_connection()
        try:
            return int(self._tello.get_yaw())
        except Exception as exc:
            raise RuntimeError("读取航向角失败：请确认无人机连接正常。") from exc

    def stream_on(self) -> None:
        """Enable the drone camera stream."""
        self._require_connection()
        try:
            with self._command_lock:
                self._tello.streamon()
            self.streaming = True
        except Exception as exc:
            raise RuntimeError("开启视频流失败：请确认无人机连接正常。") from exc

    def stream_off(self) -> None:
        """Disable the drone camera stream."""
        if not self.connected or self._tello is None:
            print("关闭视频流命令未执行：无人机尚未连接。")
            return
        try:
            with self._command_lock:
                self._tello.streamoff()
            self.streaming = False
        except Exception as exc:
            raise RuntimeError("关闭视频流失败：请确认无人机连接正常。") from exc

    def get_frame(self) -> Any:
        """Return the latest frame from the drone camera stream."""
        self._require_connection()
        if not self.streaming:
            raise RuntimeError("读取画面失败：请先调用 stream_on() 开启视频流。")
        try:
            import cv2
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。") from exc
        try:
            frame_reader = self._tello.get_frame_read()
            frame = frame_reader.frame
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            raise RuntimeError("读取画面失败：请确认视频流已经开启。") from exc

    def _create_tello(self) -> Any:
        """Create a djitellopy.Tello instance only when hardware is needed."""
        try:
            from djitellopy import Tello
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 djitellopy 依赖：请先安装 requirements.txt。") from exc
        try:
            return Tello(host=self.host)
        except TypeError:
            tello = Tello()
            if self.host != "192.168.10.1":
                raise RuntimeError(
                    "当前 djitellopy 版本不支持指定 host，无法安全连接多台 TT。"
                )
            return tello

    def _send_command_for_response(self, command: str, timeout: int) -> str:
        """Use djitellopy's sole response receiver; never read its UDP socket here."""
        if self._tello is None:
            raise RuntimeError("[Tello] socket error: SDK client is not initialized")
        try:
            with self._command_lock:
                response = self._tello.send_command_with_return(command, timeout=timeout)
        except OSError as exc:
            raise RuntimeError(f"[Tello] socket error: {exc}") from exc
        except Exception as exc:
            raise RuntimeError(f"[Tello] socket error: {exc}") from exc
        return str(response).strip()

    def _require_ok_response(self, response: str) -> None:
        """Accept only the exact SDK acknowledgement used by the ground-truth probe."""
        normalized = response.strip().lower()
        if self._is_timeout_response(response):
            raise RuntimeError(
                f"[Tello] UDP timeout waiting for command response from "
                f"{self.host}:{self.SDK_PORT}"
            )
        if normalized == "ok":
            return
        if normalized.startswith("error") or "rejected" in normalized:
            raise RuntimeError(f"[Tello] SDK command rejected: {response!r}")
        raise RuntimeError(f"[Tello] unexpected response to command: {response!r}")

    def _query_integer(self, command: str) -> int:
        response = self._send_command_for_response(
            command, timeout=self.COMMAND_TIMEOUT_SECONDS
        )
        if self._is_timeout_response(response):
            raise RuntimeError(f"UDP timeout waiting for {command}")
        try:
            return int(response)
        except (TypeError, ValueError) as exc:
            raise RuntimeError(f"unexpected response to {command}: {response!r}") from exc

    @staticmethod
    def _is_timeout_response(response: str) -> bool:
        normalized = str(response).strip().lower()
        return "did not receive a response" in normalized or "timeout" in normalized

    def _require_connection(self) -> None:
        """Raise a Chinese error if no aircraft connection is available."""
        if not self.connected or self._tello is None:
            raise RuntimeError("无人机尚未连接：请先连接 RoboMaster TT / Tello 的 Wi-Fi。")


TelloAdapter = TelloDroneAdapter
