"""Safety-wrapped tools exposed to the rule-based Agent scheduler."""

from threading import Event, Lock, Thread, current_thread
from typing import Dict, Optional, Tuple

from control.follow_control import FollowController
from control.follow_session import FollowSession
from drone.drone_adapter import DroneAdapter
from drone.safety import SafetyManager
from vision.target_detect import TargetDetector


class AgentTools:
    """Safe task-level operations available to the Agent.

    The Agent may call these task tools, but it never receives direct access to
    djitellopy or Tello.send_rc_control. Real-time RC output stays inside this
    safety wrapper and is always limited by SafetyManager.
    """

    def __init__(
        self,
        drone: DroneAdapter,
        safety_manager: SafetyManager,
        detector: TargetDetector,
        follow_controller: FollowController,
        config: Optional[dict] = None,
        mode_label: str = "REAL",
        frame_width: int = 640,
        frame_height: int = 480,
    ) -> None:
        self._drone = drone
        self._safety_manager = safety_manager
        self._detector = detector
        self._follow_controller = follow_controller
        self._config = config or {}
        self._mode_label = mode_label
        self.frame_width = frame_width
        self.frame_height = frame_height

        self.current_mode = "未连接"
        self.connected = False
        self.airborne = False
        self.streaming = False
        self._stop_event = Event()
        self._control_lock = Lock()
        self._active_session: Optional[FollowSession] = None
        self._session_thread: Optional[Thread] = None

    def connect(self) -> None:
        """Connect through the selected DroneAdapter."""
        if self.connected:
            return
        self._drone.connect()
        self.connected = True
        self.current_mode = "待机"

    def get_status(self) -> Dict[str, object]:
        """Return battery, height, and current task mode."""
        self._require_connection()
        return {
            "battery": self._drone.get_battery(),
            "height": self._drone.get_height(),
            "mode": self.current_mode,
        }

    def can_start_task(self) -> Tuple[bool, str]:
        """Check the takeoff battery threshold through SafetyManager."""
        self._require_connection()
        battery = self._drone.get_battery()
        if self._safety_manager.can_takeoff(battery):
            return True, f"当前电量 {battery}%，允许开始任务。"
        return (
            False,
            f"电量 {battery}% 低于安全起飞阈值 "
            f"{self._safety_manager.config.min_battery_takeoff}%，禁止起飞。",
        )

    def start_follow_task(self) -> bool:
        """Check safety, take off, and run the visual Agent follow session."""
        self._require_connection()
        if self.airborne or self._active_session is not None:
            print("任务已经在运行，请先停止当前任务。")
            return False

        allowed, message = self.can_start_task()
        if not allowed:
            print(message)
            return False

        session = FollowSession(
            drone=self._drone,
            safety_manager=self._safety_manager,
            detector=self._detector,
            follow_controller=self._follow_controller,
            config=self._config,
            mode_label=self._mode_label,
            window_name=str(
                self._config.get("agent_window_name", "DroneUmbrella Agent Follow")
            ),
            state_label="AGENT",
            allow_pause=True,
            stop_event=self._stop_event,
        )
        self._active_session = session
        self.current_mode = "跟随任务"
        self._stop_event.clear()
        self._session_thread = Thread(
            target=self._run_follow_session,
            args=(session,),
            daemon=True,
        )
        self._session_thread.start()
        print("Agent 跟随任务已启动，可继续输入“停止任务”或“急停”。")
        return True

    def is_task_active(self) -> bool:
        """Return whether a visual follow session is currently active."""
        return self._active_session is not None

    def start_task_after_confirmation(self) -> bool:
        """Compatibility wrapper for starting the visual follow task."""
        return self.start_follow_task()

    def _legacy_follow_task_error_cleanup(self) -> None:
        """Keep cleanup code available for older call paths."""
        try:
            self._safe_zero_output()
            if self.airborne:
                self._safe_land()
            self._stop_stream()
        except RuntimeError:
            raise

    def stop_task(self) -> None:
        """Stop following and land safely."""
        active_session = self._active_session
        self._stop_event.set()
        if active_session is not None and hasattr(active_session, "request_stop"):
            active_session.request_stop()
        self._safe_zero_output()
        self._wait_for_active_session()
        if self.airborne:
            self._safe_land()
        self._stop_stream()
        self.current_mode = "待机" if self.connected else "未连接"
        print("当前任务已停止，无人机已降落。")

    def emergency_stop(self) -> None:
        """Immediately stop follow output and request the active task to land."""
        active_session = self._active_session
        self._stop_event.set()
        if active_session is not None and hasattr(active_session, "request_emergency_stop"):
            active_session.request_emergency_stop()
        self._safe_zero_output()
        self._wait_for_active_session()
        self.current_mode = "急停"
        print("急停已执行：当前控制输出已清零，跟随任务已停止。")

    def close(self) -> None:
        """Stop active work, land if needed, and release adapter resources."""
        if self._active_session is not None or self.airborne:
            self.stop_task()
        else:
            self._stop_event.set()
            self._stop_stream()
        if self.connected:
            self._drone.stop()
        self.connected = False
        self.current_mode = "已退出"

    def _send_safe_rc(
        self,
        left_right: int,
        forward_backward: int,
        up_down: int,
        yaw: int,
    ) -> None:
        """Limit every channel before sending through DroneAdapter."""
        limited = self._safety_manager.limit_rc_command(
            left_right,
            forward_backward,
            up_down,
            yaw,
        )
        with self._control_lock:
            if not self._stop_event.is_set() or limited == (0, 0, 0, 0):
                self._drone.move_rc(*limited)

    def _safe_zero_output(self) -> None:
        """Send a safety-limited zero command when connected."""
        if not self.connected:
            return
        try:
            self._send_safe_rc(0, 0, 0, 0)
        except RuntimeError as exc:
            print(f"控制输出清零失败：{exc}")

    def _safe_land(self) -> None:
        """Land through DroneAdapter and update state."""
        try:
            self._drone.land()
        finally:
            self.airborne = False

    def _stop_stream(self) -> None:
        """Stop the video stream if it is active."""
        if not self.streaming:
            return
        try:
            self._drone.stream_off()
        finally:
            self.streaming = False

    def _is_following(self) -> bool:
        """Return whether a follow session is active."""
        return self._active_session is not None

    def _require_connection(self) -> None:
        """Reject task tools until the adapter is connected."""
        if not self.connected:
            raise RuntimeError("Agent 尚未连接无人机。")

    def _run_follow_session(self, session: FollowSession) -> None:
        """Run the active visual follow session and publish final state."""
        try:
            result = session.run()
            self.airborne = result.airborne
            self.streaming = result.streaming
            self.current_mode = result.state
            print(f"Agent 跟随任务结束，当前状态：{self.current_mode}")
        except Exception as exc:
            self.current_mode = "异常保护"
            print(f"Agent 跟随任务异常：{exc}")
        finally:
            self.airborne = False
            self.streaming = False
            if self._active_session is session:
                self._active_session = None

    def _wait_for_active_session(self) -> None:
        """Wait briefly for the background follow session to finish cleanup."""
        thread = self._session_thread
        if thread is None or thread is current_thread():
            return
        if thread.is_alive():
            thread.join(timeout=10.0)
        if not thread.is_alive():
            self._session_thread = None
