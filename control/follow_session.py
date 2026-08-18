"""Shared single-drone visual follow session."""

from collections import deque
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from threading import Event, Lock, Timer
from time import monotonic, sleep
from typing import Any, Callable, Dict, Optional, Tuple

from control.fixed_demo import FixedDemoManeuver, FixedDemoProgress
from control.follow_control import FollowController, RCCommand
from control.features import build_features
from control.kernel.arbitration import ArbitrationEngine, FollowTickOutcome
from control.kernel.features import ArbitrationContext
from control.kernel.phases import KernelPhase
from control.kernel.session import KernelSession
from control.manual_control import ManualControlController
from control.motion_arbiter import MotionArbiter, MotionContext
from control.obstacle_avoidance import AvoidanceDecision, ObstacleAvoidancePlanner
from control.target_search import TargetSearchController
from drone.drone_adapter import DroneAdapter
from drone.front_tof import FrontToFMonitor
from drone.safety import SafetyManager
from vision.camera import CameraStream
from vision.detector_protocol import DetectorProtocol
from vision.obstacle_detect import DistanceOnlyObstacleDetector, ObstacleResult
from vision.reid_enrollment import TargetLockTracker


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
        detector: DetectorProtocol,
        follow_controller: FollowController,
        config: Dict[str, object],
        mode_label: str,
        window_name: Optional[str] = None,
        state_label: str = "FOLLOW",
        allow_pause: bool = False,
        stop_event: Optional[Event] = None,
        pre_follow_maneuver: Optional[FixedDemoManeuver] = None,
        obstacle_detector: Optional[DistanceOnlyObstacleDetector] = None,
        obstacle_planner: Optional[ObstacleAvoidancePlanner] = None,
        motion_arbiter: Optional[MotionArbiter] = None,
        initial_target_lock_frames: int = 0,
        initial_target_lock_timeout_seconds: float = 30.0,
        pre_takeoff_confirmation: Optional[Callable[[Dict[str, object]], bool]] = None,
        window_takeoff_confirmation: bool = False,
        enable_target_search: Optional[bool] = None,
    ) -> None:
        self.drone = drone
        self.safety_manager = safety_manager
        self.detector = detector
        self.follow_controller = follow_controller
        self.pre_follow_maneuver = pre_follow_maneuver
        # 生产路径统一使用 motion_arbiter 作为唯一避障管线（包含检测器、规划器和
        # 日志）。单独传入 obstacle_detector/obstacle_planner 只用于直接装配
        # （如测试）时的回退路径；两者同时传入时 arbiter 优先。
        self.obstacle_detector = obstacle_detector
        self.obstacle_planner = obstacle_planner
        self.motion_arbiter = motion_arbiter
        self.manual_controller = ManualControlController.from_config(
            config, safety_manager
        )
        obstacle_config = config.get("obstacle", {}) if isinstance(config, dict) else {}
        front_tof_enabled = (
            (
                self.motion_arbiter is not None
                and isinstance(obstacle_config, dict)
                and bool(obstacle_config.get("front_tof_enabled", False))
            )
            or (
                self.manual_controller.config.enabled
                and self.manual_controller.config.front_tof_guard_enabled
            )
        )
        self.front_tof_monitor = (
            FrontToFMonitor.from_config(drone, config) if front_tof_enabled else None
        )
        if self.motion_arbiter is not None and self.front_tof_monitor is not None:
            self.motion_arbiter.set_front_tof_provider(self.front_tof_monitor.snapshot)
        self.initial_target_lock_frames = max(0, int(initial_target_lock_frames))
        self.initial_target_lock_timeout_seconds = max(
            1.0, float(initial_target_lock_timeout_seconds)
        )
        self.pre_takeoff_confirmation = pre_takeoff_confirmation
        self.window_takeoff_confirmation = bool(window_takeoff_confirmation)
        self.config = config
        self.mode_label = mode_label
        self.window_name = window_name or str(
            config.get("console_window_name", "PhantomFilmer Follow")
        )
        self.state_label = state_label
        self.allow_pause = allow_pause
        self.stop_event = stop_event or Event()
        self._lifecycle_lock = Lock()
        self.display_enabled = bool(config.get("display_console_camera", True))
        self.frame_failure_limit = int(config.get("frame_failure_limit", 30))
        self.height_failure_limit = max(
            1, int(config.get("height_failure_limit", 10))
        )
        self.height_filter_window = max(
            1, int(config.get("height_filter_window", 5))
        )
        self.height_max_valid_cm = max(
            self.safety_manager.config.max_height_cm,
            int(config.get("height_max_valid_cm", 500)),
        )
        self.control_interval = self._read_control_interval(config)
        self.min_control_hz = float(config.get("min_control_hz", 8.0))
        self.target_search = TargetSearchController(
            config,
            min_height_cm=self.safety_manager.config.min_height_cm,
            max_height_cm=self.safety_manager.config.max_height_cm,
        )
        # 默认只为经过地面锁定的旧流程开启搜索；ReID 直接起飞流程可显式开启，
        # 从而把“是否搜索”与“起飞前是否必须看到目标”彻底解耦。
        search_requested = (
            self.initial_target_lock_frames > 0
            if enable_target_search is None
            else bool(enable_target_search)
        )
        self.target_search_enabled = (
            search_requested and self.target_search.enabled
        )
        self.search_reason = ""

        self.camera: Optional[CameraStream] = None
        self.session_state = "IDLE"
        self.airborne = False
        self.streaming = False
        self.paused = False
        self.emergency_stop = False
        self.last_command = RCCommand()
        self.last_battery: Optional[int] = None
        self.last_height: Optional[int] = None
        self.last_raw_height: Optional[int] = None
        self.last_yaw: Optional[int] = None
        self._height_samples: deque[int] = deque(maxlen=self.height_filter_window)
        self.last_obstacle_result: Optional[ObstacleResult] = None
        self.last_avoidance_decision: Optional[AvoidanceDecision] = None
        self.fps = 0.0
        self.control_hz = 0.0
        self._control_rate_warning_shown = False
        self._state_label_font = None
        self._manual_reacquire_tracker: Optional[TargetLockTracker] = None
        self._manual_output_lock = Lock()
        self._manual_watchdog: Optional[Timer] = None
        self._manual_watchdog_generation = 0
        self._manual_mode_switch_suppressed_until = 0.0
        self._features = build_features(
            follow_controller=self.follow_controller,
            safety_manager=self.safety_manager,
            target_search=self.target_search,
            search_enabled=self.target_search_enabled,
            motion_arbiter=self.motion_arbiter,
            manual_controller=self.manual_controller,
            mode_label=self.mode_label,
        )
        self._arbitration = ArbitrationEngine(
            features=self._features,
            follow_controller=self.follow_controller,
            mode_label=self.mode_label,
        )
        # 内核门面：工作循环 / 唯一 RC 发射点 / fail-safe 都收拢在 KernelSession。
        self._kernel = KernelSession(self)

    @property
    def console_state(self) -> str:
        """Compatibility property used by earlier console tests."""
        return self.session_state

    @console_state.setter
    def console_state(self, value: str) -> None:
        self.session_state = value

    def run(self) -> FollowSessionResult:
        """Start stream, take off, show the follow window, and clean up safely.

        The lifecycle work loop, single RC emission seam, and fail-safe live in
        ``KernelSession``; this facade keeps the constructor signature and return
        type unchanged so callers and tests are unaffected.
        """
        return self._kernel.run()

    def _wait_for_initial_target_lock(self) -> Dict[str, object]:
        """Keep the aircraft grounded until fresh ReID matches are stable."""
        tracker = TargetLockTracker(self.initial_target_lock_frames)
        started_at = monotonic()
        frame_failures = 0
        detect_failures = 0
        self.session_state = "GROUND_TARGET_LOCK"
        print(
            "无人机保持在地面，正在从摄像头画面识别目标人物。"
            f"需要连续确认 {tracker.required_frames} 帧。"
        )
        while not self.stop_event.is_set():
            if monotonic() - started_at >= self.initial_target_lock_timeout_seconds:
                print(
                    "地面识别超时，未起飞。"
                    "请调整人物位置、光线或参考照片。"
                )
                return {}

            frame = self._read_frame()
            if frame is None:
                frame_failures += 1
                if frame_failures >= self.frame_failure_limit:
                    print("地面识别时连续无法读取视频帧，未起飞。")
                    return {}
                sleep(self.control_interval)
                continue

            frame_failures = 0
            try:
                result = self.detector.detect(frame)
            except Exception as exc:
                detect_failures += 1
                print(
                    "地面 ReID 检测异常"
                    f"（{detect_failures}/{self.frame_failure_limit}）：{exc}"
                )
                if detect_failures >= self.frame_failure_limit:
                    print("地面 ReID 检测器连续异常，未起飞。")
                    return {}
                sleep(self.control_interval)
                continue
            detect_failures = 0

            locked = tracker.observe(result)
            if self.display_enabled:
                import cv2

                preview = self.detector.draw_debug(frame, result)
                similarity = result.get("similarity")
                similarity_text = (
                    "N/A" if similarity is None else f"{float(similarity):.3f}"
                )
                cv2.putText(
                    preview,
                    f"GROUND LOCK {tracker.progress} similarity={similarity_text}",
                    (20, max(64, preview.shape[0] - 48)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    (0, 255, 255),
                    2,
                )
                cv2.putText(
                    preview,
                    "Q cancel | aircraft remains grounded",
                    (20, max(88, preview.shape[0] - 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    1,
                )
                cv2.imshow(self.window_name, preview)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("已取消地面目标识别，未起飞。")
                    self.session_state = "STOPPED"
                    return {}

            if locked:
                print(
                    "目标人物已稳定锁定："
                    f"similarity={float(result.get('similarity') or 0.0):.3f}。"
                )
                return result
            sleep(self.control_interval)
        self.session_state = "STOPPED"
        return {}

    def _verify_fresh_target_before_takeoff(self) -> bool:
        """Reject stale authorization if the target moved during confirmation."""
        frame = self._read_frame()
        if frame is None:
            return False
        try:
            result = self.detector.detect(frame)
        except Exception as exc:
            print(f"起飞前最终 ReID 检查失败：{exc}")
            return False
        return TargetLockTracker(required_frames=1).observe(result)

    def _wait_for_window_takeoff_confirmation(self) -> Dict[str, object]:
        """Keep one OpenCV window alive while Y/Q authorizes or cancels takeoff."""
        import cv2

        timeout = max(
            1.0,
            float(
                self.config.get(
                    "reid_confirmation_timeout_seconds",
                    self.initial_target_lock_timeout_seconds,
                )
            ),
        )
        started_at = monotonic()
        frame_failures = 0
        self.session_state = "GROUND_TAKEOFF_CONFIRMATION"
        print("目标人物已锁定。请在视频窗口按 Y 确认起飞，按 Q 取消。")

        while not self.stop_event.is_set():
            if monotonic() - started_at >= timeout:
                print("起飞确认超时，无人机保持在地面。")
                self.session_state = "TAKEOFF_CANCELLED"
                return {}

            frame = self._read_frame()
            if frame is None:
                frame_failures += 1
                if frame_failures >= self.frame_failure_limit:
                    print("等待起飞确认时连续无法读取视频帧，无人机保持在地面。")
                    return {}
                cv2.waitKey(1)
                sleep(self.control_interval)
                continue

            frame_failures = 0
            try:
                result = self.detector.detect(frame)
            except Exception as exc:
                print(f"等待起飞确认时 ReID 检测失败：{exc}")
                return {}

            target_is_fresh = TargetLockTracker(required_frames=1).observe(result)
            preview = self.detector.draw_debug(frame, result)
            status = (
                "TARGET LOCKED - Y: take off | Q: cancel"
                if target_is_fresh
                else "TARGET LOST - move into view | Q: cancel"
            )
            color = (0, 255, 0) if target_is_fresh else (0, 165, 255)
            cv2.putText(
                preview,
                status,
                (20, max(36, preview.shape[0] - 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                color,
                2,
            )
            cv2.imshow(self.window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q")):
                print("已取消起飞，无人机保持在地面。")
                self.session_state = "TAKEOFF_CANCELLED"
                return {}
            if key in (ord("y"), ord("Y")):
                if not target_is_fresh:
                    print("当前目标未通过最终身份检查，请重新进入画面后再按 Y。")
                else:
                    cv2.putText(
                        preview,
                        "TAKEOFF AUTHORIZED - keep clear",
                        (20, 36),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 255),
                        2,
                    )
                    cv2.imshow(self.window_name, preview)
                    cv2.waitKey(1)
                    return result
            sleep(self.control_interval)

        self.session_state = "TAKEOFF_CANCELLED"
        return {}

    def _wait_for_takeoff_stabilization(self, duration: float) -> bool:
        """Keep ReID and the shared window active while takeoff stabilizes."""
        keep_detecting = self.initial_target_lock_frames > 0
        if not self.display_enabled and not keep_detecting:
            sleep(duration)
            return True

        cv2 = None
        if self.display_enabled:
            import cv2

        deadline = monotonic() + max(0.0, duration)
        frame_failures = 0
        detect_failures = 0
        while monotonic() < deadline and not self.stop_event.is_set():
            frame = self._read_frame()
            if frame is None:
                frame_failures += 1
                if frame_failures >= self.frame_failure_limit:
                    print("起飞稳定阶段连续无法读取视频帧，准备安全降落。")
                    self.session_state = "FRAME_LOST_LANDING"
                    return False
            else:
                frame_failures = 0
                preview = frame
                if keep_detecting:
                    try:
                        result = self.detector.detect(frame)
                        detect_failures = 0
                        if self.display_enabled:
                            preview = self.detector.draw_debug(frame, result)
                    except Exception as exc:
                        detect_failures += 1
                        print(
                            "起飞稳定阶段 ReID 检测异常"
                            f"（{detect_failures}/{self.frame_failure_limit}）：{exc}"
                        )
                        if detect_failures >= self.frame_failure_limit:
                            self.session_state = "FRAME_LOST_LANDING"
                            return False
                if self.display_enabled:
                    overlay_text = (
                        "TAKING OFF - ReID remains active"
                        if keep_detecting
                        else "TAKING OFF"
                    )
                    cv2.putText(
                        preview,
                        overlay_text,
                        (20, max(36, preview.shape[0] - 48)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )
                    cv2.putText(
                        preview,
                        "q: land | e: emergency land",
                        (20, max(68, preview.shape[0] - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        1,
                    )
                    cv2.imshow(self.window_name, preview)
            if self.display_enabled:
                action = self.handle_key(cv2.waitKey(1) & 0xFF)
                if action in ("stop", "emergency"):
                    return False
            sleep(self.control_interval)
        return not self.stop_event.is_set()

    def _verify_takeoff_height(self) -> bool:
        """Confirm takeoff from several telemetry samples instead of one transient value."""
        timeout = max(
            0.2,
            float(self.config.get("takeoff_height_verify_timeout_seconds", 5.0)),
        )
        minimum_height = max(
            10,
            int(self.config.get("takeoff_height_min_cm", 20)),
        )
        required_readings = max(
            1,
            int(self.config.get("takeoff_height_stable_readings", 3)),
        )
        sample_interval = max(
            0.05,
            float(self.config.get("takeoff_height_sample_interval_seconds", 0.2)),
        )
        deadline = monotonic() + timeout
        valid_readings = 0
        last_height: Optional[int] = None
        self.session_state = "VERIFYING_TAKEOFF_HEIGHT"
        print(
            f"正在确认起飞高度：需要连续 {required_readings} 次达到 "
            f"{minimum_height} cm。"
        )

        while monotonic() < deadline and not self.stop_event.is_set():
            height = self._read_height()
            last_height = height
            if height is not None and height >= minimum_height:
                valid_readings += 1
            else:
                valid_readings = 0

            if valid_readings >= required_readings:
                self.session_state = "TAKEOFF_HEIGHT_READY"
                print(f"起飞高度确认成功：{height} cm。")
                return True

            if self.display_enabled:
                import cv2

                frame = self._read_frame()
                if frame is not None:
                    preview = frame
                    height_text = "N/A" if height is None else str(height)
                    cv2.putText(
                        preview,
                        f"VERIFY HEIGHT {valid_readings}/{required_readings} | {height_text} cm",
                        (20, max(36, preview.shape[0] - 48)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255),
                        2,
                    )
                    cv2.putText(
                        preview,
                        "q: land | e: emergency land",
                        (20, max(68, preview.shape[0] - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        1,
                    )
                    cv2.imshow(self.window_name, preview)
                action = self.handle_key(cv2.waitKey(1) & 0xFF)
                if action in ("stop", "emergency"):
                    return False
            sleep(sample_interval)

        if self.stop_event.is_set():
            if self.session_state != "EMERGENCY_STOP":
                self.session_state = "STOPPED"
        else:
            shown_height = "N/A" if last_height is None else f"{last_height} cm"
            print(f"起飞高度确认超时（最后读数：{shown_height}），准备安全降落。")
            self.session_state = "STOPPED"
            self._safe_zero_output()
        return False

    def _reach_base_hover_height(self) -> bool:
        """Reach base altitude without making climb permission depend on ReID."""
        try:
            target_height = int(self.config.get("base_hover_height_cm", 0))
        except (TypeError, ValueError):
            target_height = 0
        if target_height <= 0:
            return True

        minimum_height = self.safety_manager.config.min_height_cm
        maximum_height = self.safety_manager.config.max_height_cm
        if not minimum_height <= target_height <= maximum_height:
            print(
                f"基础悬停高度 {target_height} cm 不在安全范围 "
                f"[{minimum_height}, {maximum_height}] cm 内，准备降落。"
            )
            self.session_state = "BASE_HEIGHT_FAILED"
            return False

        tolerance = max(
            2,
            int(self.config.get("base_hover_height_tolerance_cm", 5)),
        )
        vertical_speed = max(
            1,
            min(
                self.safety_manager.config.max_rc_speed,
                abs(int(self.config.get("base_hover_vertical_speed", 20))),
            ),
        )
        required_stable_readings = max(
            1,
            int(self.config.get("base_hover_stable_readings", 3)),
        )
        keep_detecting = self.initial_target_lock_frames > 0
        stable_readings = 0
        height_failures = 0
        frame_failures = 0
        detect_failures = 0
        next_battery_check = monotonic()
        self.session_state = "REACHING_BASE_HEIGHT"
        print(f"正在到达基础悬停高度：{target_height} cm（不受 ReID 结果和总时限影响）。")

        while not self.stop_event.is_set():
            now = monotonic()
            if now >= next_battery_check:
                battery = self._read_battery()
                next_battery_check = now + 1.0
                if battery is not None and self.safety_manager.should_land(battery):
                    print(f"升高阶段电量已降至 {battery}%，准备安全降落。")
                    self.session_state = "LOW_BATTERY_LANDING"
                    self._safe_zero_output()
                    return False
            height = self._read_height()
            if height is None:
                height_failures += 1
                self._safe_zero_output()
                if height_failures >= self.frame_failure_limit:
                    print("连续无法读取飞行高度，准备安全降落。")
                    self.session_state = "BASE_HEIGHT_FAILED"
                    return False
                sleep(self.control_interval)
                continue

            height_failures = 0
            if height > maximum_height:
                print(
                    f"飞行高度 {height} cm 已超过安全上限 {maximum_height} cm，"
                    "准备安全降落。"
                )
                self.session_state = "HEIGHT_LIMIT_LANDING"
                self._safe_zero_output()
                return False

            preview = None
            result: Optional[Dict[str, object]] = None
            frame = None
            if keep_detecting or self.display_enabled:
                frame = self._read_frame()
                if frame is None:
                    frame_failures += 1
                    if frame_failures >= self.frame_failure_limit:
                        print("升高阶段连续无法读取视频帧，准备安全降落。")
                        self.session_state = "FRAME_LOST_LANDING"
                        self._safe_zero_output()
                        return False
                else:
                    frame_failures = 0
                    try:
                        result = self.detector.detect(frame)
                        detect_failures = 0
                        if self.display_enabled:
                            preview = self.detector.draw_debug(frame, result)
                    except Exception as exc:
                        detect_failures += 1
                        print(
                            "升高阶段 ReID 检测异常"
                            f"（{detect_failures}/{self.frame_failure_limit}）：{exc}"
                        )
                        if detect_failures >= self.frame_failure_limit:
                            self.session_state = "FRAME_LOST_LANDING"
                            self._safe_zero_output()
                            return False
                        preview = frame

            height_error = target_height - height
            if abs(height_error) <= tolerance:
                stable_readings += 1
                command = self.follow_controller.hover()
            else:
                stable_readings = 0
                command = RCCommand(
                    up_down=vertical_speed if height_error > 0 else -vertical_speed
                )
            # 基础高度爬升只执行垂直控制。首次可靠识别目标之前，顶部前向
            # ToF 不参与运动仲裁，避免在起飞阶段触发横移或转向。
            self.send_command(command)

            if self.display_enabled:
                import cv2

                if preview is not None:
                    identity_found = bool(result and result.get("found"))
                    identity_text = "ReID FOUND" if identity_found else "ReID NOT FOUND"
                    cv2.putText(
                        preview,
                        f"BASE HEIGHT {height}/{target_height} cm | {identity_text}",
                        (20, max(36, preview.shape[0] - 48)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255) if identity_found else (0, 165, 255),
                        2,
                    )
                    cv2.putText(
                        preview,
                        "q: land | e: emergency land",
                        (20, max(68, preview.shape[0] - 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (255, 255, 255),
                        1,
                    )
                    cv2.imshow(self.window_name, preview)
                action = self.handle_key(cv2.waitKey(1) & 0xFF)
                if action in ("stop", "emergency"):
                    return False

            if stable_readings >= required_stable_readings:
                self._safe_zero_output()
                self.session_state = "BASE_HEIGHT_READY"
                print(f"基础悬停高度已稳定：{height} cm。")
                return True
            sleep(self.control_interval)

        self._safe_zero_output()
        if self.stop_event.is_set():
            if self.session_state != "EMERGENCY_STOP":
                self.session_state = "STOPPED"
        return False

    def _pre_follow_should_abort(self) -> bool:
        """Stop a predefined maneuver for stop, emergency, or manual takeover."""
        return (
            self.stop_event.is_set()
            or self.emergency_stop
            or self.manual_controller.active
        )

    def _fixed_demo_is_avoiding(self) -> bool:
        """Hold the route timer while obstacle avoidance is overriding it."""
        if self.last_obstacle_result is None:
            return False
        return bool(self.last_obstacle_result.found)

    def _show_pre_follow_progress(self, progress: FixedDemoProgress) -> bool:
        """Display the raw camera during the route and keep m/q/e responsive."""
        if not self.display_enabled:
            return not self._pre_follow_should_abort()

        frame = self._read_frame()
        if frame is None:
            return not self._pre_follow_should_abort()

        import cv2

        phase = "悬停稳定" if progress.settling else progress.step.name
        cv2.putText(
            frame,
            f"FIXED DEMO {progress.step_index}/{progress.step_count}: {phase}",
            (20, 36),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            "m: manual takeover, q: stop + land, e: emergency + land",
            (20, 68),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        cv2.imshow(self.window_name, frame)
        action = self.handle_key(cv2.waitKey(1) & 0xFF)
        return action not in ("stop", "emergency")

    def _prepare_detector(self) -> None:
        """Preload optional detector models while the drone is still grounded."""
        prepare_method = getattr(self.detector, "prepare", None)
        if callable(prepare_method):
            prepare_method()

    def process_detection(
        self,
        target_result: Dict[str, object],
        frame_width: int,
        frame_height: int,
        height_cm: Optional[int] = None,
        now: Optional[float] = None,
        yaw_deg: Optional[int] = None,
    ) -> Tuple[RCCommand, str]:
        """Convert one detection result into a safe command and lost-target action."""
        if self.emergency_stop:
            return self.follow_controller.hover(), "emergency"

        if self.paused:
            self.session_state = "PAUSED"
            return self.follow_controller.hover(), "paused"

        if self.target_search_enabled:
            # ReID 搜索状态机自己负责配置的总超时；持续清掉旧的8秒丢失计时，
            # 避免两套计时互相冲突、搜索尚未完成就提前降落。
            self.safety_manager.update_target_lost(True)
            decision = self.target_search.update(
                target_result,
                frame_width,
                frame_height,
                height_cm,
                monotonic() if now is None else now,
                yaw_deg=yaw_deg,
            )
            if decision is not None:
                self.session_state = decision.state
                self.search_reason = decision.reason
                if decision.action == "land":
                    self.session_state = "TARGET_LOST_LANDING"
                return decision.command, decision.action

            self.session_state = "FOLLOWING"
            self.search_reason = ""
            command = self.follow_controller.compute_command(
                target_result, frame_width, frame_height
            )
            self.target_search.observe_target(
                target_result, frame_width, frame_height, command
            )
            return command, "keep"

        lost_action = self.safety_manager.update_target_lost(bool(target_result.get("found")))
        if lost_action == "land":
            self.session_state = "TARGET_LOST_LANDING"
            return self.follow_controller.hover(), lost_action

        if not target_result.get("found") or lost_action == "hover":
            return self.follow_controller.hover(), lost_action

        self.session_state = "FOLLOWING"
        return self.follow_controller.compute_command(target_result, frame_width, frame_height), lost_action

    def send_command(self, command: RCCommand) -> None:
        """Send a command through the kernel's single RC emission seam."""
        if self.manual_controller.active:
            # Revalidate the short lease at the actual emission boundary.  The
            # watchdog uses the same lock, so an expired command can never be
            # re-sent after the watchdog has emitted its zero.
            with self._manual_output_lock:
                if self.manual_controller.active:
                    command = self.manual_controller.command_for(
                        now=monotonic(),
                        height_cm=self.last_height,
                        front_tof_snapshot=self._front_tof_snapshot(),
                    )
                self._kernel._emit(command)
            return
        self._kernel._emit(command)

    def send_motion_command(self, command: RCCommand) -> None:
        """Apply the shared obstacle arbiter before sending an autonomous command."""
        if self.manual_controller.active:
            # A fixed-demo callback can observe ``m`` between two route ticks.
            # Its ``finally`` block still sends one last zero command through
            # this seam; never let that cleanup call restart an old autonomous
            # obstacle plan after the operator has taken control.
            self._safe_zero_output()
            return
        if self.motion_arbiter is None:
            self.send_command(command)
            return
        frame = self._read_frame()
        if frame is None:
            self.send_command(self.follow_controller.hover())
            return
        decision = self.motion_arbiter.decide(
            desired_command=command,
            frame=frame,
            # 预飞段（fixed-demo 固定航线）尚未进入目标跟随，故意不提供目标区域：
            # 让避障检测器观察整个画面，避免目标排除逻辑误用于目标可能尚未进入
            # 视野的阶段。进入正常跟随后由 _loop 传入真实检测结果。
            context=MotionContext(mode=self.mode_label, target_result={"found": False}),
        )
        self.last_obstacle_result = decision.observation
        self.last_avoidance_decision = decision
        self.send_command(decision.command)

    def handle_key(self, key: int) -> Optional[str]:
        """Handle global safety keys plus manual takeover and motion keys."""
        if key in (ord("e"), ord("E")):
            self._cancel_manual_watchdog(force_hover=True)
            self.emergency_stop = True
            self.paused = False
            self.session_state = "EMERGENCY_STOP"
            self._safe_zero_output()
            print("急停：已清零控制输出，准备安全降落。")
            return "emergency"

        if key in (ord("q"), ord("Q")):
            self._cancel_manual_watchdog(force_hover=True)
            self.session_state = "STOPPED"
            self._safe_zero_output()
            print("跟随停止：准备降落。")
            return "stop"

        if self.allow_pause and key == ord("p"):
            self._cancel_manual_watchdog(force_hover=True)
            self.paused = not self.paused
            if self._manual_reacquire_tracker is not None:
                self._manual_reacquire_tracker.reset()
            self.session_state = (
                "PAUSED"
                if self.paused
                else ("MANUAL" if self.manual_controller.active else "FOLLOWING")
            )
            self._safe_zero_output()
            if not self.paused and self.motion_arbiter is not None:
                # 恢复时强制下一帧重新检测，避免复用暂停前可能已过期的观测。
                self.motion_arbiter.invalidate_observation()
            print("跟随已暂停。" if self.paused else "跟随已恢复。")
            return None

        if (
            key in (ord("m"), ord("M"))
            and self.manual_controller.available
            and not self.paused
        ):
            now = monotonic()
            if now < self._manual_mode_switch_suppressed_until:
                # Key-repeat events extend the quiet period.  A long-held ``m``
                # therefore cannot toggle again as soon as the first fixed
                # cooldown expires; the operator must release, wait, and press.
                self._manual_mode_switch_suppressed_until = (
                    now
                    + self.manual_controller.config.mode_switch_debounce_seconds
                )
                return None
            if self.manual_controller.active:
                self._leave_manual_mode()
                print("已退出手动控制，重新搜索并确认目标。")
            elif self._enter_manual_mode():
                print("已进入手动控制；无新方向按键时自动悬停。")
            return None

        if self.manual_controller.active:
            if self.paused:
                return None
            with self._manual_output_lock:
                consumed = self.manual_controller.handle_key(key, monotonic())
                if consumed:
                    if key == ord(" "):
                        self._cancel_manual_watchdog_locked(force_hover=False)
                        self._kernel._emit(self.follow_controller.hover())
                    else:
                        self._arm_manual_watchdog_locked()

        return None

    def request_stop(self) -> None:
        """Request a normal stop from outside the OpenCV window loop."""
        with self._lifecycle_lock:
            if self.session_state != "EMERGENCY_STOP":
                self.session_state = "STOPPED"
            self.stop_event.set()

    def request_emergency_stop(self) -> None:
        """Request an emergency stop from outside the OpenCV window loop."""
        with self._lifecycle_lock:
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
        obstacle_detector = self.obstacle_detector
        if obstacle_detector is None and self.motion_arbiter is not None:
            obstacle_detector = self.motion_arbiter.detector
        if obstacle_detector is not None:
            debug_frame = obstacle_detector.draw_debug(debug_frame, self.last_obstacle_result)
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

        # Keep the camera image uncluttered: detector annotations, guide lines,
        # and the single top-right STATE label remain; the former black
        # telemetry/parameter panel is intentionally removed.
        del command, battery, height
        display_state = self._display_state()
        state_colors = {
            "FOLLOW": (0, 255, 0),
            "SEARCH": (0, 255, 255),
            "OBSTACLE": (0, 165, 255),
            "MANUAL": (255, 120, 0),
            "CONTROL_READY": (255, 160, 0),
            "HOVER": (255, 160, 0),
            "LANDING": (0, 0, 255),
            "PAUSED": (160, 160, 160),
        }
        return self._draw_state_label(
            debug_frame,
            f"STATE: {self._display_state_chinese(display_state)}",
            state_colors[display_state],
        )

    @staticmethod
    def _display_state_chinese(display_state: str) -> str:
        """Translate the stable internal state key for the camera overlay."""
        return {
            "FOLLOW": "跟随",
            "SEARCH": "搜索",
            "OBSTACLE": "避障",
            "MANUAL": "手动控制",
            "CONTROL_READY": "等待选择",
            "HOVER": "悬停",
            "LANDING": "降落",
            "PAUSED": "暂停",
        }.get(display_state, "悬停")

    def _draw_state_label(
        self, frame: Any, text: str, bgr_color: Tuple[int, int, int]
    ) -> Any:
        """Draw one larger Chinese-capable state label at the top-right."""
        import numpy as np
        from PIL import Image, ImageDraw, ImageFont

        if self._state_label_font is None:
            candidates = (
                "/System/Library/Fonts/STHeiti Medium.ttc",
                "/System/Library/Fonts/PingFang.ttc",
                "/System/Library/Fonts/Supplemental/Arial Unicode.ttf",
                "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
                "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
            )
            font_path = next((path for path in candidates if Path(path).is_file()), None)
            if font_path is not None:
                self._state_label_font = ImageFont.truetype(font_path, 30)
            else:
                # The fallback keeps the flight UI alive on an unexpected host;
                # production macOS uses STHeiti and therefore renders Chinese.
                self._state_label_font = ImageFont.load_default()

        rgb = frame[:, :, ::-1].copy()
        image = Image.fromarray(rgb)
        draw = ImageDraw.Draw(image)
        red, green, blue = bgr_color[2], bgr_color[1], bgr_color[0]
        left, _top, right, _bottom = draw.textbbox(
            (0, 0), text, font=self._state_label_font, stroke_width=3
        )
        text_width = right - left
        text_x = max(12, image.width - text_width - 16)
        draw.text(
            (text_x, 12),
            text,
            font=self._state_label_font,
            fill=(red, green, blue),
            stroke_width=3,
            stroke_fill=(0, 0, 0),
        )
        return np.asarray(image)[:, :, ::-1].copy()

    def _display_state(self) -> str:
        """Return the single user-facing state that currently owns the aircraft."""
        session_state = str(self.session_state or "").upper()
        if "LANDING" in session_state or session_state in {
            "EMERGENCY_STOP",
            "FAILSAFE",
        }:
            return "LANDING"
        if self.paused or session_state == "PAUSED":
            return "PAUSED"
        if session_state == "CONTROL_READY":
            return "CONTROL_READY"
        if self.manual_controller.active or session_state == "MANUAL":
            return "MANUAL"
        if session_state in {"REACQUIRE_VERIFY", "TARGET_REACQUIRED"}:
            return "SEARCH"

        avoidance_state = str(
            getattr(self.last_avoidance_decision, "state", "") or ""
        ).upper()
        if session_state == "OBSTACLE_FIRST" or avoidance_state in {
            "BRAKING",
            "CAUTION",
            "SCAN",
            "AVOIDING",
            "RECOVERING",
            "FAILSAFE",
        }:
            return "OBSTACLE"
        if self.target_search.searching:
            return "SEARCH"
        if session_state == "FOLLOWING":
            return "FOLLOW"
        return "HOVER"

    def _wait_for_control_selection(self) -> Optional[str]:
        """Hover indefinitely at base height until M/manual or A/auto is chosen."""
        if not self.display_enabled:
            print("手动/自动选择需要启用摄像头窗口；当前已停止并准备降落。")
            self.session_state = "STOPPED"
            self._safe_zero_output()
            return None

        import cv2

        self.session_state = "CONTROL_READY"
        self._safe_zero_output()
        print("已到达基础悬停高度。按 m 进入手动，按 a 进入自动；将持续悬停等待。")
        frame_failures = 0
        height_failures = 0
        next_battery_check = monotonic()

        while not self.stop_event.is_set():
            loop_started_at = monotonic()
            now = loop_started_at
            if now >= next_battery_check:
                battery = self._read_battery()
                next_battery_check = now + 1.0
                if battery is not None and self.safety_manager.should_land(battery):
                    print(f"等待控制选择时电量已降至 {battery}%，准备安全降落。")
                    self.session_state = "LOW_BATTERY_LANDING"
                    self._safe_zero_output()
                    return None

            height = self._read_height()
            if height is None:
                height_failures += 1
                self._safe_zero_output()
                if height_failures >= self.height_failure_limit:
                    print("等待控制选择时连续无法读取高度，准备安全降落。")
                    self.session_state = "HEIGHT_SENSOR_LANDING"
                    return None
            else:
                height_failures = 0
                if not self.safety_manager.check_height(height):
                    print(f"等待控制选择时高度达到 {height} cm，准备安全降落。")
                    self.session_state = "HEIGHT_LIMIT_LANDING"
                    self._safe_zero_output()
                    return None

            frame = self._read_frame()
            if frame is None:
                frame_failures += 1
                self._safe_zero_output()
                if frame_failures >= self.frame_failure_limit:
                    print("等待控制选择时连续无法读取视频流，准备安全降落。")
                    self.session_state = "FRAME_LOST_LANDING"
                    return None
                sleep(self.control_interval)
                continue

            frame_failures = 0
            self._safe_zero_output()
            preview = frame.copy()
            shown_height = "N/A" if height is None else f"{height} cm"
            shown_battery = (
                "N/A" if self.last_battery is None else f"{self.last_battery}%"
            )
            cv2.putText(
                preview,
                "M: MANUAL   A: AUTO   Q: LAND   E: EMERGENCY",
                (20, max(36, preview.shape[0] - 48)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                preview,
                f"HOVERING | height={shown_height} battery={shown_battery}",
                (20, max(68, preview.shape[0] - 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
            )
            preview = self._draw_state_label(
                preview, "STATE: 等待选择", (255, 160, 0)
            )
            cv2.imshow(self.window_name, preview)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("m"), ord("M")):
                self.paused = False
                if self._enter_manual_mode():
                    print("已选择手动控制。")
                    return "manual"
            elif key in (ord("a"), ord("A")):
                self.paused = False
                self.manual_controller.disable()
                self._safe_zero_output()
                self.session_state = "FOLLOWING"
                print("已选择自动控制。")
                return "auto"
            elif key in (ord("q"), ord("Q"), ord("e"), ord("E")):
                action = self.handle_key(key)
                if action in ("stop", "emergency"):
                    return None

            loop_elapsed = monotonic() - loop_started_at
            sleep(max(0.0, self.control_interval - loop_elapsed))

        self._safe_zero_output()
        if self.session_state != "EMERGENCY_STOP":
            self.session_state = "STOPPED"
        return None

    def _enter_manual_mode(self) -> bool:
        """Atomically zero autonomous output and give control to the operator."""
        self._cancel_manual_watchdog(force_hover=True)
        now = monotonic()
        if not self.manual_controller.enable(now):
            return False
        self._manual_mode_switch_suppressed_until = (
            now + self.manual_controller.config.mode_switch_debounce_seconds
        )
        self.paused = False
        self._manual_reacquire_tracker = None
        # Stop the previous owner before any detector/search/arbiter reset can
        # block on model or log cleanup.  This is the actual ownership handoff.
        self.session_state = "MANUAL"
        self._safe_zero_output()
        # Discard every autonomous state machine at the ownership boundary.
        # Manual ticks never call these modules, and exiting manual starts from
        # another clean state plus an explicit consecutive-ReID gate.
        reset_detector = getattr(self.detector, "reset", None)
        if callable(reset_detector):
            reset_detector()
        self.follow_controller.reset()
        self.target_search.reset()
        if self.motion_arbiter is not None:
            self.motion_arbiter.reset(self.mode_label)
        else:
            reset_obstacle = getattr(self.obstacle_detector, "reset", None)
            if callable(reset_obstacle):
                reset_obstacle()
            if self.obstacle_planner is not None:
                self.obstacle_planner.reset()
        self.last_obstacle_result = None
        self.last_avoidance_decision = None
        self._arbitration.reset()
        self.safety_manager.update_target_lost(True)
        return True

    def _leave_manual_mode(self) -> None:
        """Hover, discard stale autonomy state, and restart fresh acquisition."""
        self._manual_mode_switch_suppressed_until = (
            monotonic()
            + self.manual_controller.config.mode_switch_debounce_seconds
        )
        self._cancel_manual_watchdog(force_hover=True)
        self._safe_zero_output()
        self.manual_controller.disable()
        reset_detector = getattr(self.detector, "reset", None)
        if callable(reset_detector):
            reset_detector()
        self.follow_controller.reset()
        self.target_search.reset()
        if self.motion_arbiter is not None:
            self.motion_arbiter.reset(self.mode_label)
        else:
            reset_obstacle = getattr(self.obstacle_detector, "reset", None)
            if callable(reset_obstacle):
                reset_obstacle()
            if self.obstacle_planner is not None:
                self.obstacle_planner.reset()
        self.last_obstacle_result = None
        self.last_avoidance_decision = None
        self._arbitration.reset()
        self.safety_manager.update_target_lost(True)
        self._manual_reacquire_tracker = TargetLockTracker(
            self.manual_controller.config.reacquire_frames
        )
        self.search_reason = "manual released; reacquiring target"
        self.session_state = "REACQUIRE_VERIFY"

    def _arm_manual_watchdog_locked(self) -> None:
        """Arm one exact deadman timer; caller owns ``_manual_output_lock``."""
        self._cancel_manual_watchdog_locked(force_hover=False)
        generation = self._manual_watchdog_generation
        timer = Timer(
            self.manual_controller.config.command_timeout_seconds,
            self._manual_watchdog_expired,
            args=(generation,),
        )
        timer.daemon = True
        self._manual_watchdog = timer
        timer.start()

    def _manual_watchdog_expired(self, generation: int) -> None:
        """Collapse manual output even if camera or telemetry work is blocked."""
        with self._manual_output_lock:
            if generation != self._manual_watchdog_generation:
                return
            self._manual_watchdog = None
            if not self.manual_controller.active:
                return
            self.manual_controller.force_hover("manual command timed out")
            try:
                self._kernel._emit(self.follow_controller.hover())
            except RuntimeError as exc:
                print(f"手动控制看门狗清零失败：{exc}")

    def _cancel_manual_watchdog(self, *, force_hover: bool) -> None:
        with self._manual_output_lock:
            self._cancel_manual_watchdog_locked(force_hover=force_hover)

    def _cancel_manual_watchdog_locked(self, *, force_hover: bool) -> None:
        """Cancel a pending timer; caller owns ``_manual_output_lock``."""
        self._manual_watchdog_generation += 1
        timer = self._manual_watchdog
        self._manual_watchdog = None
        if timer is not None:
            timer.cancel()
        if force_hover and self.manual_controller.active:
            self.manual_controller.force_hover()

    @staticmethod
    def _manual_target_result() -> Dict[str, object]:
        """Neutral result used while expensive target inference is suspended."""
        return {
            "found": False,
            "is_predicted": False,
            "ambiguous": False,
            "center": None,
            "area": 0.0,
            "area_ratio": 0.0,
            "bbox": None,
            "candidates": [],
            "visual_objects": [],
        }

    def _front_tof_snapshot(self) -> Optional[object]:
        monitor = self.front_tof_monitor
        return None if monitor is None else monitor.snapshot()

    def _loop(self) -> None:
        """Run the real-time visual follow loop."""
        cv2 = None
        if self.display_enabled:
            import cv2

        frame_failures = 0
        detect_failures = 0
        height_failures = 0
        engine_failures = 0
        stats_started_at = monotonic()
        frame_counter = 0
        command_counter = 0
        next_manual_battery_check = monotonic()

        while True:
            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in (
                    "LOW_BATTERY_LANDING",
                    "HEIGHT_LIMIT_LANDING",
                    "TARGET_LOST_LANDING",
                    "FRAME_LOST_LANDING",
                    "HEIGHT_SENSOR_LANDING",
                ):
                    self.session_state = "STOPPED"
                break

            loop_started_at = monotonic()
            battery = self.last_battery
            height = self.last_height
            if self.manual_controller.active:
                if loop_started_at >= next_manual_battery_check:
                    battery = self._read_battery()
                    next_manual_battery_check = monotonic() + 1.0
                height = self._read_height()
                if height is None:
                    height_failures += 1
                    self._safe_zero_output()
                    print(
                        "TOF 离地高度无效，正在重试"
                        f"（{height_failures}/{self.height_failure_limit}）。"
                    )
                    if height_failures >= self.height_failure_limit:
                        print("连续无法获得有效 TOF 离地高度，准备安全降落。")
                        self.session_state = "HEIGHT_SENSOR_LANDING"
                        break
                    sleep(self.control_interval)
                    continue
                height_failures = 0
                if battery is not None and self.safety_manager.should_land(battery):
                    print(f"电量已降至 {battery}%，准备安全降落。")
                    self.session_state = "LOW_BATTERY_LANDING"
                    self._safe_zero_output()
                    break
                if not self.safety_manager.check_height(height):
                    print(
                        f"飞行高度 {height} cm 已超过安全上限 "
                        f"{self.safety_manager.config.max_height_cm} cm，准备安全降落。"
                    )
                    self.session_state = "HEIGHT_LIMIT_LANDING"
                    self._safe_zero_output()
                    break

            frame = self._read_frame()
            if frame is None:
                if self._manual_reacquire_tracker is not None:
                    # "Consecutive" ReID confirmation cannot bridge a missing
                    # video frame; restart the proof after any stream gap.
                    self._manual_reacquire_tracker.reset()
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
            if self.manual_controller.active:
                # Manual control must remain responsive even when ReID inference
                # is expensive.  Camera/telemetry stay live, but target inference
                # resumes only after manual release and a fresh state reset.
                target_result = self._manual_target_result()
                detect_failures = 0
            else:
                try:
                    target_result = self.detector.detect(frame)
                except Exception as exc:
                    if self._manual_reacquire_tracker is not None:
                        # An inference gap is just as discontinuous as a lost
                        # frame, so matches on either side must not be combined.
                        self._manual_reacquire_tracker.reset()
                    # 检测器异常（如 ReID 推理失败）不中断整个任务：先零输出并重试，
                    # 连续失败达到帧数上限后按视频丢失同样的安全策略降落。
                    detect_failures += 1
                    print(
                        f"目标检测异常（{detect_failures}/{self.frame_failure_limit}）：{exc}"
                    )
                    self._safe_zero_output()
                    if detect_failures >= self.frame_failure_limit:
                        print("检测器连续异常，准备安全降落。")
                        self.session_state = "FRAME_LOST_LANDING"
                        break
                    sleep(0.05)
                    continue
                detect_failures = 0
            yaw = self._read_yaw()
            tracker = self._manual_reacquire_tracker
            if tracker is not None and (self.paused or self.emergency_stop):
                outcome = FollowTickOutcome(
                    command=self.follow_controller.hover(),
                    state="PAUSED" if self.paused else "",
                )
            elif tracker is not None:
                if tracker.observe(target_result):
                    self._manual_reacquire_tracker = None
                    outcome = FollowTickOutcome(
                        command=self.follow_controller.hover(),
                        state="TARGET_REACQUIRED",
                        reason="manual release target confirmed",
                    )
                else:
                    outcome = FollowTickOutcome(
                        command=self.follow_controller.hover(),
                        state="REACQUIRE_VERIFY",
                        reason=f"manual release ReID {tracker.progress}",
                    )
            else:
                # 每 tick 的仲裁由 ArbitrationEngine 唯一决定：暂停/急停、手动、
                # 首次目标门控、避障接管、自动跟随或目标搜索。
                try:
                    outcome = self._arbitration.arbitrate(
                        ArbitrationContext(
                            phase=KernelPhase.FOLLOW,
                            target_result=target_result,
                            frame=frame,
                            frame_width=frame_width,
                            frame_height=frame_height,
                            height_cm=height,
                            yaw_deg=yaw,
                            paused=self.paused,
                            emergency=self.emergency_stop,
                            stop_requested=self.stop_event.is_set(),
                            front_tof_snapshot=self._front_tof_snapshot(),
                            now=monotonic(),
                        )
                    )
                except Exception as exc:
                    engine_failures += 1
                    print(
                        f"仲裁引擎异常"
                        f"（{engine_failures}/{self.frame_failure_limit}）：{exc}"
                    )
                    self._kernel._failsafe(exc)
                    if engine_failures >= self.frame_failure_limit:
                        print("仲裁引擎连续异常，准备安全降落。")
                        self.session_state = "FRAME_LOST_LANDING"
                        break
                    sleep(0.05)
                    continue
            engine_failures = 0
            command = outcome.command
            if outcome.state:
                self.session_state = outcome.state
            if outcome.reason:
                self.search_reason = outcome.reason
            if outcome.obstacle_ran:
                self.last_obstacle_result = outcome.obstacle_observation
                self.last_avoidance_decision = outcome.avoidance_decision
            if outcome.requires_landing:
                self.session_state = outcome.landing_state
                self._safe_zero_output()
                break

            if not self.manual_controller.active:
                battery = self._read_battery()
                height = self._read_height()
                if height is None:
                    height_failures += 1
                    self._safe_zero_output()
                    print(
                        "TOF 离地高度无效，正在重试"
                        f"（{height_failures}/{self.height_failure_limit}）。"
                    )
                    if height_failures >= self.height_failure_limit:
                        print("连续无法获得有效 TOF 离地高度，准备安全降落。")
                        self.session_state = "HEIGHT_SENSOR_LANDING"
                        break
                    sleep(self.control_interval)
                    continue
                height_failures = 0

            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in (
                    "LOW_BATTERY_LANDING",
                    "HEIGHT_LIMIT_LANDING",
                    "TARGET_LOST_LANDING",
                    "FRAME_LOST_LANDING",
                    "HEIGHT_SENSOR_LANDING",
                ):
                    self.session_state = "STOPPED"
                break

            if battery is not None and self.safety_manager.should_land(battery):
                print(f"电量已降至 {battery}%，准备安全降落。")
                self.session_state = "LOW_BATTERY_LANDING"
                self._safe_zero_output()
                break

            if height is not None and not self.safety_manager.check_height(height):
                print(
                    f"飞行高度 {height} cm 已超过安全上限 "
                    f"{self.safety_manager.config.max_height_cm} cm，准备安全降落。"
                )
                self.session_state = "HEIGHT_LIMIT_LANDING"
                self._safe_zero_output()
                break

            if outcome.lost_land:
                print("目标长时间丢失，准备安全降落。")
                self._safe_zero_output()
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
                if self.control_hz < self.min_control_hz:
                    if not self._control_rate_warning_shown:
                        print(
                            f"控制率偏低：{self.control_hz:.1f} Hz < {self.min_control_hz:.1f} Hz，"
                            "检测或显示处理可能过慢，请检查性能。"
                        )
                        self._control_rate_warning_shown = True
                else:
                    self._control_rate_warning_shown = False

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

    def _prepare_front_tof(self) -> None:
        """Verify the optional top/front ToF while the aircraft is grounded."""
        if self.front_tof_monitor is not None:
            print("正在检查顶部前向 ToF 距离模块...")
            self.front_tof_monitor.prepare()
            print("顶部前向 ToF 已就绪：60 cm 内将触发 BLOCKED。")

    def _start_front_tof(self) -> None:
        """Begin cached distance polling after takeoff."""
        if self.front_tof_monitor is not None:
            self.front_tof_monitor.start()

    def _stop_front_tof(self) -> None:
        """Stop polling before SDK landing/stream commands."""
        if self.front_tof_monitor is not None:
            self.front_tof_monitor.stop()

    def _reset_tracking_state(self) -> None:
        """Clear detector and controller state before every independent follow task."""
        self._cancel_manual_watchdog(force_hover=True)
        self.manual_controller.reset()
        self._manual_reacquire_tracker = None
        reset_method = getattr(self.detector, "reset", None)
        if callable(reset_method):
            reset_method()
        self.follow_controller.reset()
        if self.motion_arbiter is not None:
            self.motion_arbiter.reset(self.mode_label)
        else:
            obstacle_reset_method = getattr(self.obstacle_detector, "reset", None)
            if callable(obstacle_reset_method):
                obstacle_reset_method()
            if self.obstacle_planner is not None:
                self.obstacle_planner.reset()
        self.last_obstacle_result = None
        self.last_avoidance_decision = None
        self._height_samples.clear()
        self.last_height = None
        self.last_raw_height = None
        self.last_yaw = None
        self.target_search.reset()
        self._arbitration.reset()
        self.search_reason = ""

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
        """Read non-blocking runtime battery telemetry when the adapter supports it."""
        try:
            cached_reader = getattr(self.drone, "get_cached_battery", None)
            reader = cached_reader if callable(cached_reader) else self.drone.get_battery
            self.last_battery = int(reader())
        except RuntimeError as exc:
            print(f"读取电量失败：{exc}")
            self.last_battery = None
        return self.last_battery

    def _read_yaw(self) -> Optional[int]:
        """Read cached flight-controller yaw without making it a landing dependency."""
        try:
            self.last_yaw = int(self.drone.get_yaw())
        except RuntimeError:
            self.last_yaw = None
        return self.last_yaw

    def _read_height(self) -> Optional[int]:
        """Read and median-filter downward TOF ground clearance."""
        try:
            raw_height = int(self.drone.get_height())
            self.last_raw_height = raw_height
            if raw_height <= 0 or raw_height > self.height_max_valid_cm:
                print(
                    f"忽略异常 TOF 离地高度：{raw_height} cm；"
                    f"有效范围为 1～{self.height_max_valid_cm} cm。"
                )
                self.last_height = None
                return None
            self._height_samples.append(raw_height)
            self.last_height = int(round(median(self._height_samples)))
        except (RuntimeError, TypeError, ValueError) as exc:
            print(f"读取 TOF 离地高度失败：{exc}")
            self.last_height = None
        return self.last_height

    def _safe_zero_output(self) -> None:
        """Send zero through the kernel's single bounded RC emission seam."""
        try:
            self._kernel._emit(self.follow_controller.hover())
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
