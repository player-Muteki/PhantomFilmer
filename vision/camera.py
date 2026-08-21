"""Camera stream wrapper based on the unified DroneAdapter interface."""

from typing import Any, Optional

from drone.drone_adapter import DroneAdapter


class CameraStream:
    """Read camera frames through DroneAdapter instead of a hardware SDK."""

    def __init__(
        self,
        drone: DroneAdapter,
        width: int = 640,
        height: int = 480,
        *,
        manage_stream: bool = True,
    ) -> None:
        self.drone = drone
        self.width = width
        self.height = height
        self.manage_stream = bool(manage_stream)
        self.running = False

    def start(self) -> None:
        """Start the drone video stream."""
        try:
            if self.manage_stream:
                self.drone.stream_on()
            self.running = True
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError("开启摄像头视频流失败：请检查无人机连接。") from exc

    def read_frame(self) -> Optional[Any]:
        """Return the latest frame from the drone camera."""
        if not self.running:
            raise RuntimeError("摄像头视频流尚未开启。")
        try:
            return self.drone.get_frame()
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError("读取摄像头画面失败：请检查视频流状态。") from exc

    def stop(self) -> None:
        """Stop the drone video stream."""
        if not self.running:
            return
        try:
            if self.manage_stream:
                self.drone.stream_off()
        finally:
            self.running = False
