"""Shared single-drone visual follow session."""

from dataclasses import dataclass
from threading import Event
from time import monotonic, sleep
from typing import Any, Dict, Optional, Tuple

from control.follow_control import FollowController, RCCommand
from drone.drone_adapter import DroneAdapter
from drone.safety import SafetyManager
from vision.camera import CameraStream
from vision.detector_factory import VisionDetector


@dataclass
class FollowSessionResult:
    """Final state returned after a follow session exits."""

    state: str
    airborne: bool
    streaming: bool


class FollowSession:
    """Run one shared visual target-following task."""

    def __init__(
        self,
        drone: DroneAdapter,
        safety_manager: SafetyManager,
        detector: VisionDetector,
        follow_controller: FollowController,
        config: Dict[str, object],
        mode_label: str,
        window_name: Optional[str] = None,
        state_label: str = "FOLLOW",
        allow_pause: bool = False,
        stop_event: Optional[Event] = None,
    ) -> None:
        self.drone = drone
        self.safety_manager = safety_manager
        self.detector = detector
        self.follow_controller = follow_controller
        self.config = config
        self.mode_label = mode_label
        self.window_name = window_name or str(
            config.get("agent_window_name", "DroneUmbrella Follow")
        )
        self.state_label = state_label
        self.allow_pause = allow_pause
        self.stop_event = stop_event or Event()
        self.display_enabled = bool(config.get("display_agent_camera", True))
        self.frame_failure_limit = int(config.get("frame_failure_limit", 30))
        self.control_interval = self._read_control_interval(config)

        self.camera: Optional[CameraStream] = None
        self.session_state = "IDLE"
        self.airborne = False
        self.streaming = False
        self.paused = False
        self.emergency_stop = False
        self.last_command = RCCommand()
        self.last_battery: Optional[int] = None
        self.last_height: Optional[int] = None
        self.fps = 0.0
        self.control_hz = 0.0

    def run(self) -> FollowSessionResult:
        """Start stream, take off, show the follow window, and clean up safely."""
        try:
            self._start_camera()
            self.drone.takeoff()
            sleep(2)
            height = self._read_height()
            if height is not None and height < 10:
                print("未检测到起飞高度，跟随任务未启动。")
                self.session_state = "STOPPED"
                return FollowSessionResult(
                    state=self.session_state,
                    airborne=self.airborne,
                    streaming=self.streaming,
                )

            self.airborne = True
            self.session_state = "FOLLOWING"
            print(f"跟随任务已启动，当前运行模式：{self.mode_label}")
            if self.allow_pause:
                print("窗口按键：p 暂停/继续，q 停止并降落，e 急停并降落。")
            else:
                print("窗口按键：q 停止并降落，e 急停并降落。")
            self._loop()
        except KeyboardInterrupt:
            print("收到 Ctrl+C，正在安全停止跟随任务。")
            self.session_state = "STOPPED"
        finally:
            self._safe_zero_output()
            if self.airborne:
                self._safe_land()
            self._stop_camera()
            self._destroy_window()

        return FollowSessionResult(
            state=self.session_state,
            airborne=self.airborne,
            streaming=self.streaming,
        )

    def process_detection(
        self,
        target_result: Dict[str, object],
        frame_width: int,
        frame_height: int,
    ) -> Tuple[RCCommand, str]:
        """Convert one detection result into a safe command and lost-target action."""
        lost_action = self.safety_manager.update_target_lost(bool(target_result.get("found")))
        if lost_action == "land":
            self.session_state = "TARGET_LOST_LANDING"
            return self.follow_controller.hover(), lost_action

        if self.emergency_stop:
            return self.follow_controller.hover(), "emergency"

        if self.paused:
            self.session_state = "PAUSED"
            return self.follow_controller.hover(), "paused"

        if not target_result.get("found") or lost_action == "hover":
            return self.follow_controller.hover(), lost_action

        self.session_state = "FOLLOWING"
        return self.follow_controller.compute_command(target_result, frame_width, frame_height), lost_action

    def send_command(self, command: RCCommand) -> None:
        """Send a command through SafetyManager and DroneAdapter."""
        if self.emergency_stop or self.paused or self.stop_event.is_set():
            command = self.follow_controller.hover()

        limited = self.safety_manager.limit_rc_command(*command.as_tuple())
        self.last_command = RCCommand(*limited)
        self.drone.move_rc(*self.last_command.as_tuple())

    def handle_key(self, key: int) -> Optional[str]:
        """Handle q/e and optional p window keys."""
        if self.allow_pause and key == ord("p"):
            self.paused = not self.paused
            self.session_state = "PAUSED" if self.paused else "FOLLOWING"
            self._safe_zero_output()
            print("跟随已暂停。" if self.paused else "跟随已恢复。")
            return None

        if key == ord("e"):
            self.emergency_stop = True
            self.paused = False
            self.session_state = "EMERGENCY_STOP"
            self._safe_zero_output()
            print("急停：已清零控制输出，准备安全降落。")
            return "emergency"

        if key == ord("q"):
            self.session_state = "STOPPED"
            self._safe_zero_output()
            print("跟随停止：准备降落。")
            return "stop"

        return None

    def request_stop(self) -> None:
        """Request a normal stop from outside the OpenCV window loop."""
        if self.session_state != "EMERGENCY_STOP":
            self.session_state = "STOPPED"
        self.stop_event.set()

    def request_emergency_stop(self) -> None:
        """Request an emergency stop from outside the OpenCV window loop."""
        self.emergency_stop = True
        self.paused = False
        self.session_state = "EMERGENCY_STOP"
        self.stop_event.set()
        self._safe_zero_output()

    def draw_debug_frame(
        self,
        frame: Any,
        target_result: Dict[str, object],
        command: RCCommand,
        battery: Optional[int],
        height: Optional[int],
    ) -> Any:
        """Draw target, status, and command data on a frame."""
        import cv2

        debug_frame = self.detector.draw_debug(frame, target_result)
        frame_height, frame_width = debug_frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)
        vertical_dead_zone_px = int(frame_height * self.follow_controller.vertical_dead_zone_ratio / 2)
        if vertical_dead_zone_px > 0:
            upper_y = max(0, frame_center[1] - vertical_dead_zone_px)
            lower_y = min(frame_height - 1, frame_center[1] + vertical_dead_zone_px)
            cv2.line(debug_frame, (0, upper_y), (frame_width, upper_y), (80, 80, 255), 1)
            cv2.line(debug_frame, (0, lower_y), (frame_width, lower_y), (80, 80, 255), 1)

        if target_result.get("found") and target_result.get("center") is not None:
            target_center = target_result["center"]
            tx, ty = target_center  # type: ignore[misc]
            cv2.line(debug_frame, frame_center, (int(tx), int(ty)), (0, 255, 255), 2)

        panel_width = min(frame_width - 24, 430)
        cv2.rectangle(debug_frame, (12, 84), (12 + panel_width, 326), (0, 0, 0), -1)
        target_text = "FOUND" if target_result.get("found") else "LOST"
        battery_text = f"{battery}%" if battery is not None else "N/A"
        height_text = f"{height} cm" if height is not None else "N/A"
        lr, fb, ud, yaw = command.as_tuple()
        debug = self.follow_controller.last_debug
        key_text = "KEYS: p pause/resume, q stop+land, e emergency"
        if not self.allow_pause:
            key_text = "KEYS: q stop+land, e emergency"
        lines = (
            f"MODE: {self.mode_label}",
            f"{self.state_label}: {self.session_state}",
            f"TARGET: {target_text}",
            f"BATTERY: {battery_text}",
            f"HEIGHT: {height_text}",
            f"RC: lr={lr} fb={fb} ud={ud} yaw={yaw}",
            f"FPS: {self.fps:.1f}  CTRL_HZ: {self.control_hz:.1f}",
            f"X: {debug.target_center_x}/{debug.frame_center_x} err={debug.horizontal_error} ({debug.horizontal_error_ratio:.2f})",
            f"Y: {debug.target_center_y}/{debug.frame_center_y} err={debug.vertical_error} ({debug.vertical_error_ratio:.2f})",
            f"AREA: {debug.area_ratio:.3f}  STATE: {debug.target_state}",
            key_text,
        )
        for index, line in enumerate(lines):
            cv2.putText(
                debug_frame,
                line,
                (20, 108 + index * 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 255),
                1,
            )
        return debug_frame

    def _loop(self) -> None:
        """Run the real-time visual follow loop."""
        cv2 = None
        if self.display_enabled:
            import cv2

        frame_failures = 0
        stats_started_at = monotonic()
        frame_counter = 0
        command_counter = 0

        while True:
            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in ("LOW_BATTERY_LANDING", "TARGET_LOST_LANDING", "FRAME_LOST_LANDING"):
                    self.session_state = "STOPPED"
                break

            loop_started_at = monotonic()
            frame = self._read_frame()
            if frame is None:
                frame_failures += 1
                self._safe_zero_output()
                print(f"未读取到视频帧，正在重试（{frame_failures}/{self.frame_failure_limit}）。")
                if frame_failures >= self.frame_failure_limit:
                    print("连续无法获取无人机视频流，准备安全降落。")
                    self.session_state = "FRAME_LOST_LANDING"
                    break
                sleep(0.05)
                continue

            frame_failures = 0
            frame_counter += 1
            frame_height, frame_width = frame.shape[:2]
            target_result = self.detector.detect(frame)
            command, lost_action = self.process_detection(target_result, frame_width, frame_height)

            battery = self._read_battery()
            height = self._read_height()
            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in ("LOW_BATTERY_LANDING", "TARGET_LOST_LANDING", "FRAME_LOST_LANDING"):
                    self.session_state = "STOPPED"
                break

            if battery is not None and self.safety_manager.should_land(battery):
                print(f"电量已降至 {battery}%，准备安全降落。")
                self.session_state = "LOW_BATTERY_LANDING"
                break

            if lost_action == "land":
                print("目标长时间丢失，准备安全降落。")
                break

            self.send_command(command)
            command_counter += 1
            now = monotonic()
            elapsed = now - stats_started_at
            if elapsed >= 0.5:
                self.fps = frame_counter / elapsed
                self.control_hz = command_counter / elapsed
                frame_counter = 0
                command_counter = 0
                stats_started_at = now

            if self.display_enabled:
                debug_frame = self.draw_debug_frame(frame, target_result, self.last_command, battery, height)
                cv2.imshow(self.window_name, debug_frame)
                key = cv2.waitKey(1) & 0xFF
                action = self.handle_key(key)
                if action in ("stop", "emergency"):
                    break

            loop_elapsed = monotonic() - loop_started_at
            sleep(max(0.0, self.control_interval - loop_elapsed))

    def _start_camera(self) -> None:
        """Start the video stream through CameraStream."""
        self.camera = CameraStream(
            drone=self.drone,
            width=int(self.config.get("camera_width", 640)),
            height=int(self.config.get("camera_height", 480)),
        )
        try:
            self.camera.start()
            self.streaming = True
        except RuntimeError:
            raise
        except Exception as exc:
            raise RuntimeError(
                "无法获取无人机视频流，请检查是否已连接 RoboMaster TT / Tello Wi-Fi。"
            ) from exc

    def _read_frame(self) -> Any:
        """Read one frame without crashing on a transient failure."""
        if self.camera is None:
            return None
        try:
            return self.camera.read_frame()
        except RuntimeError as exc:
            print(str(exc))
            return None

    def _read_battery(self) -> Optional[int]:
        """Read battery, returning None when unavailable."""
        try:
            self.last_battery = self.drone.get_battery()
        except RuntimeError as exc:
            print(f"读取电量失败：{exc}")
            self.last_battery = None
        return self.last_battery

    def _read_height(self) -> Optional[int]:
        """Read height, returning None when unavailable."""
        try:
            self.last_height = self.drone.get_height()
        except RuntimeError as exc:
            print(f"读取高度失败：{exc}")
            self.last_height = None
        return self.last_height

    def _safe_zero_output(self) -> None:
        """Send a zero RC command through the same safety path."""
        zero = self.follow_controller.hover()
        limited = self.safety_manager.limit_rc_command(*zero.as_tuple())
        self.last_command = RCCommand(*limited)
        try:
            self.drone.move_rc(*self.last_command.as_tuple())
        except RuntimeError as exc:
            print(f"控制输出清零失败：{exc}")

    def _safe_land(self) -> None:
        """Land and update local airborne state."""
        try:
            self.drone.land()
        except RuntimeError as exc:
            print(f"降落失败：{exc}")
        finally:
            self.airborne = False

    def _stop_camera(self) -> None:
        """Stop stream once and update local streaming state."""
        if self.camera is None or not self.streaming:
            return
        try:
            self.camera.stop()
        except RuntimeError as exc:
            print(f"关闭视频流失败：{exc}")
        finally:
            self.streaming = False

    def _destroy_window(self) -> None:
        """Close OpenCV windows without affecting landing cleanup."""
        if not self.display_enabled:
            return
        try:
            import cv2

            cv2.destroyWindow(self.window_name)
        except Exception:
            try:
                import cv2

                cv2.destroyAllWindows()
            except Exception:
                pass

    @staticmethod
    def _read_control_interval(config: Dict[str, object]) -> float:
        """Read the control-loop interval, defaulting to about 20 Hz."""
        try:
            interval = float(config.get("control_interval", 0.05))
        except (TypeError, ValueError):
            interval = 0.05
        return max(0.02, min(0.2, interval))
