"""RoboMaster TT / Tello Talent adapter based on djitellopy.

This is the only module that imports djitellopy directly. The rest of the
project should use DroneAdapter so hardware details stay isolated here.
"""

from typing import Any, Optional

from .drone_adapter import DroneAdapter


class TelloDroneAdapter(DroneAdapter):
    """Control RoboMaster TT / Tello Talent through djitellopy.Tello."""

    def __init__(self) -> None:
        self._tello: Optional[Any] = None
        self.connected = False
        self.streaming = False

    def connect(self) -> None:
        """Connect to the drone over its Wi-Fi network."""
        if self.connected:
            return
        try:
            self._tello = self._create_tello()
            self._tello.connect()
            self.connected = True
        except Exception as exc:
            self._tello = None
            self.connected = False
            raise RuntimeError(
                "连接无人机失败：请先连接 RoboMaster TT / Tello 的 Wi-Fi，"
                "并确认无人机已开机。"
            ) from exc

    def takeoff(self) -> None:
        """Take off only after the user explicitly confirms the action."""
        self._require_connection()
        answer = input("即将起飞，请确认周围安全并输入 YES 继续：").strip()
        if answer != "YES":
            print("已取消起飞：未收到用户确认。")
            return
        try:
            self._tello.takeoff()
        except Exception as exc:
            raise RuntimeError("起飞失败：请检查电量、桨叶保护罩和飞行环境。") from exc

    def land(self) -> None:
        """Land the aircraft if connected."""
        if not self.connected or self._tello is None:
            print("降落命令未执行：无人机尚未连接。")
            return
        try:
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
                self._tello.streamoff()
        except Exception as exc:
            print(f"停止无人机时出现异常：{exc}")
        finally:
            self.connected = False
            self.streaming = False

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
            return int(self._tello.get_battery())
        except Exception as exc:
            raise RuntimeError("读取电量失败：请确认无人机连接正常。") from exc

    def get_height(self) -> int:
        """Return current flight height in centimeters."""
        self._require_connection()
        try:
            return int(self._tello.get_height())
        except Exception as exc:
            raise RuntimeError("读取高度失败：请确认无人机连接正常。") from exc

    def stream_on(self) -> None:
        """Enable the drone camera stream."""
        self._require_connection()
        try:
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
        return Tello()

    def _require_connection(self) -> None:
        """Raise a Chinese error if no aircraft connection is available."""
        if not self.connected or self._tello is None:
            raise RuntimeError("无人机尚未连接：请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
