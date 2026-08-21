"""Minimal real-device adapter required by the standalone WebUI package."""

from __future__ import annotations

import re
from threading import Lock
from typing import Any, Optional


class RealTelloAdapter:
    """Connect to one RoboMaster TT / Tello Talent over its direct Wi-Fi."""

    HOST = "192.168.10.1"
    SDK_PORT = 8889
    COMMAND_TIMEOUT_SECONDS = 5

    def __init__(self) -> None:
        self._tello: Optional[Any] = None
        self._command_lock = Lock()
        self.connected = False
        self.streaming = False
        self.last_connection_battery: Optional[int] = None

    def connect(self) -> None:
        if self.connected:
            return
        try:
            from djitellopy import Tello
        except ModuleNotFoundError as exc:
            raise RuntimeError("缺少 djitellopy，请先运行安装脚本。") from exc

        try:
            try:
                self._tello = Tello(host=self.HOST)
            except TypeError:
                self._tello = Tello()
            response = self._send_command("command", self.COMMAND_TIMEOUT_SECONDS)
            if response.strip().lower() != "ok":
                raise RuntimeError(f"无人机未确认 SDK 连接：{response!r}")
            self.connected = True
            self.last_connection_battery = self._query_integer("battery?")
        except Exception as exc:
            self._tello = None
            self.connected = False
            if isinstance(exc, RuntimeError):
                raise
            raise RuntimeError(
                f"连接 {self.HOST}:{self.SDK_PORT} 失败，请确认已连接 RMTT Wi-Fi：{exc}"
            ) from exc

    def stream_on(self) -> None:
        self._require_connection()
        try:
            with self._command_lock:
                self._tello.streamon()
            self.streaming = True
        except Exception as exc:
            raise RuntimeError("开启真机视频流失败。") from exc

    def get_frame(self) -> Any:
        self._require_connection()
        if not self.streaming:
            raise RuntimeError("视频流尚未开启。")
        try:
            import cv2

            frame = self._tello.get_frame_read().frame
            return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            raise RuntimeError("读取真机视频帧失败。") from exc

    def get_battery(self) -> int:
        self._require_connection()
        return self._query_integer("battery?")

    def get_cached_battery(self) -> int:
        self._require_connection()
        try:
            value = int(self._tello.get_battery())
        except Exception as exc:
            raise RuntimeError("读取电量遥测失败。") from exc
        if not 0 <= value <= 100:
            raise RuntimeError(f"电量遥测超出范围：{value}")
        return value

    def get_height(self) -> int:
        self._require_connection()
        try:
            return int(self._tello.get_distance_tof())
        except Exception as exc:
            raise RuntimeError("读取底部 ToF 高度失败。") from exc

    def get_front_distance_cm(self) -> Optional[float]:
        self._require_connection()
        response = self._send_command("EXT tof?", 1)
        if self._is_timeout(response):
            raise RuntimeError("读取顶部前向 ToF 超时。")
        match = re.search(r"(?:tof\s+)?(\d+)", response.strip().lower())
        if match is None:
            raise RuntimeError(f"顶部前向 ToF 返回无效：{response!r}")
        millimetres = int(match.group(1))
        if 8190 <= millimetres <= 8192:
            return None
        if not 0 < millimetres <= 1200:
            raise RuntimeError(f"顶部前向 ToF 超出范围：{millimetres} mm")
        return millimetres / 10.0

    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        self._require_connection()
        try:
            self._tello.send_rc_control(left_right, forward_backward, up_down, yaw)
        except Exception as exc:
            raise RuntimeError("清零真机控制量失败。") from exc

    def land(self) -> None:
        self._require_connection()
        try:
            with self._command_lock:
                self._tello.land()
        except Exception as exc:
            raise RuntimeError("真机降落指令失败，请立即人工观察。") from exc

    def stop(self) -> None:
        tello = self._tello
        if tello is None:
            self.connected = False
            self.streaming = False
            return
        try:
            if self.connected:
                tello.send_rc_control(0, 0, 0, 0)
            if self.streaming:
                try:
                    tello.streamoff()
                except Exception:
                    pass
            end = getattr(tello, "end", None)
            if callable(end):
                end()
        finally:
            self._tello = None
            self.connected = False
            self.streaming = False

    def _query_integer(self, command: str) -> int:
        response = self._send_command(command, self.COMMAND_TIMEOUT_SECONDS)
        if self._is_timeout(response):
            raise RuntimeError(f"等待 {command} 响应超时。")
        try:
            return int(response)
        except ValueError as exc:
            raise RuntimeError(f"{command} 返回无效：{response!r}") from exc

    def _send_command(self, command: str, timeout: int) -> str:
        if self._tello is None:
            raise RuntimeError("真机 SDK 尚未初始化。")
        try:
            with self._command_lock:
                return str(
                    self._tello.send_command_with_return(command, timeout=timeout)
                ).strip()
        except Exception as exc:
            raise RuntimeError(f"发送真机指令 {command!r} 失败：{exc}") from exc

    @staticmethod
    def _is_timeout(response: str) -> bool:
        normalized = response.strip().lower()
        return "timeout" in normalized or "did not receive a response" in normalized

    def _require_connection(self) -> None:
        if not self.connected or self._tello is None:
            raise RuntimeError("真机未连接。")
