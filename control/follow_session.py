"""Shared single-drone visual follow session."""

from dataclasses import dataclass
from threading import Event, Lock
from time import monotonic, sleep
from typing import Any, Callable, Dict, Optional, Tuple

from control.fixed_demo import FixedDemoManeuver, FixedDemoProgress
from control.follow_control import FollowController, RCCommand
from control.motion_arbiter import MotionArbiter, MotionContext
from control.obstacle_avoidance import AvoidanceDecision, ObstacleAvoidancePlanner
from drone.drone_adapter import DroneAdapter
from drone.safety import SafetyManager
from vision.camera import CameraStream
from vision.detector_protocol import DetectorProtocol
from vision.obstacle_detect import ObstacleDetector, ObstacleResult
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
        obstacle_detector: Optional[ObstacleDetector] = None,
        obstacle_planner: Optional[ObstacleAvoidancePlanner] = None,
        motion_arbiter: Optional[MotionArbiter] = None,
        initial_target_lock_frames: int = 0,
        initial_target_lock_timeout_seconds: float = 30.0,
        pre_takeoff_confirmation: Optional[Callable[[Dict[str, object]], bool]] = None,
        window_takeoff_confirmation: bool = False,
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
        self.control_interval = self._read_control_interval(config)
        self.min_control_hz = float(config.get("min_control_hz", 8.0))

        self.camera: Optional[CameraStream] = None
        self.session_state = "IDLE"
        self.airborne = False
        self.streaming = False
        self.paused = False
        self.emergency_stop = False
        self.last_command = RCCommand()
        self.last_battery: Optional[int] = None
        self.last_height: Optional[int] = None
        self.last_obstacle_result: Optional[ObstacleResult] = None
        self.last_avoidance_decision: Optional[AvoidanceDecision] = None
        self.fps = 0.0
        self.control_hz = 0.0
        self._control_rate_warning_shown = False

    @property
    def console_state(self) -> str:
        """Compatibility property used by earlier console tests."""
        return self.session_state

    @console_state.setter
    def console_state(self, value: str) -> None:
        self.session_state = value

    def run(self) -> FollowSessionResult:
        """Start stream, take off, show the follow window, and clean up safely."""
        try:
            self._reset_tracking_state()
            self._prepare_detector()
            with self._lifecycle_lock:
                if self.stop_event.is_set():
                    self.session_state = "STOPPED"
                    return FollowSessionResult(
                        state=self.session_state,
                        airborne=self.airborne,
                        streaming=self.streaming,
                    )
                self._start_camera()

            locked_result: Dict[str, object] = {}
            if self.initial_target_lock_frames > 0:
                locked_result = self._wait_for_initial_target_lock()
                if not locked_result:
                    if self.session_state != "STOPPED":
                        self.session_state = "TARGET_LOCK_FAILED"
                    return FollowSessionResult(
                        state=self.session_state,
                        airborne=self.airborne,
                        streaming=self.streaming,
                    )
                if self.window_takeoff_confirmation and self.display_enabled:
                    locked_result = self._wait_for_window_takeoff_confirmation()
                    if not locked_result:
                        if self.session_state != "TAKEOFF_CANCELLED":
                            self.session_state = "TARGET_LOCK_FAILED"
                        return FollowSessionResult(
                            state=self.session_state,
                            airborne=self.airborne,
                            streaming=self.streaming,
                        )
                elif self.pre_takeoff_confirmation is not None:
                    if not self.pre_takeoff_confirmation(locked_result):
                        print("已取消起飞：现场人员未确认目标身份。")
                        self.session_state = "TAKEOFF_CANCELLED"
                        return FollowSessionResult(
                            state=self.session_state,
                            airborne=self.airborne,
                            streaming=self.streaming,
                        )
                    if not self._verify_fresh_target_before_takeoff():
                        print(
                            "人工确认后目标已离开或身份变得模糊，"
                            "未起飞。"
                        )
                        self.session_state = "TARGET_LOCK_FAILED"
                        return FollowSessionResult(
                            state=self.session_state,
                            airborne=self.airborne,
                            streaming=self.streaming,
                        )

            with self._lifecycle_lock:
                if self.stop_event.is_set():
                    self.session_state = "STOPPED"
                    return FollowSessionResult(
                        state=self.session_state,
                        airborne=self.airborne,
                        streaming=self.streaming,
                    )
                if locked_result:
                    authorize_takeoff = getattr(self.drone, "authorize_next_takeoff", None)
                    if callable(authorize_takeoff):
                        authorize_takeoff()
                self.drone.takeoff()
                self.airborne = True
            if not self._wait_for_takeoff_stabilization(2.0):
                return FollowSessionResult(
                    state=self.session_state,
                    airborne=self.airborne,
                    streaming=self.streaming,
                )
            height = self._read_height()
            if height is not None and height < 10:
                print("未检测到起飞高度，跟随任务未启动。")
                self.session_state = "STOPPED"
                self._safe_zero_output()
                self._safe_land()
                return FollowSessionResult(
                    state=self.session_state,
                    airborne=self.airborne,
                    streaming=self.streaming,
                )
            if not self._reach_base_hover_height():
                return FollowSessionResult(
                    state=self.session_state,
                    airborne=self.airborne,
                    streaming=self.streaming,
                )

            if self.pre_follow_maneuver is not None:
                self.session_state = "FIXED_DEMO"
                print("固定演示航线已启动；航线完成后自动进入目标跟随。")
                print("窗口按键：q 停止并降落，e 急停并降落。")
                completed = self.pre_follow_maneuver.run(
                    send_command=self.send_motion_command,
                    should_abort=self._pre_follow_should_abort,
                    on_progress=self._show_pre_follow_progress,
                    is_avoiding=self._fixed_demo_is_avoiding,
                )
                if not completed:
                    if self.emergency_stop:
                        self.session_state = "EMERGENCY_STOP"
                    elif self.session_state != "STOPPED":
                        self.session_state = "STOPPED"
                    print("固定演示航线已中止，准备安全降落。")
                else:
                    self._reset_tracking_state()
                    print("固定演示航线完成，控制输出已清零，开始目标跟随。")

            if self.pre_follow_maneuver is None or completed:
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
            if self.motion_arbiter is not None:
                self.motion_arbiter.close()
            self._destroy_window()

        return FollowSessionResult(
            state=self.session_state,
            airborne=self.airborne,
            streaming=self.streaming,
        )

    def _wait_for_initial_target_lock(self) -> Dict[str, object]:
        """Keep the aircraft grounded until fresh ReID matches are stable."""
        tracker = TargetLockTracker(self.initial_target_lock_frames)
        started_at = monotonic()
        frame_failures = 0
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
                print(f"地面 ReID 检测失败，未起飞：{exc}")
                return {}

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
                    cv2.putText(
                        preview,
                        "TAKING OFF - ReID remains active",
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

    def _reach_base_hover_height(self) -> bool:
        """Reach base altitude while continuously verifying an authorized ReID target."""
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
        timeout = max(
            3.0,
            float(self.config.get("base_hover_timeout_seconds", 12.0)),
        )
        required_stable_readings = max(
            1,
            int(self.config.get("base_hover_stable_readings", 3)),
        )
        require_reid_target = self.initial_target_lock_frames > 0
        reacquire_frames = max(
            1,
            int(self.config.get("base_hover_reid_reacquire_frames", 3)),
        )
        target_lost_timeout = max(
            1.0,
            float(
                self.config.get(
                    "base_hover_target_lost_timeout_seconds",
                    self.safety_manager.config.target_lost_land_seconds,
                )
            ),
        )
        target_tracker = TargetLockTracker(reacquire_frames)
        target_ready = not require_reid_target
        target_lost_since: Optional[float] = None
        deadline = monotonic() + timeout
        stable_readings = 0
        height_failures = 0
        frame_failures = 0
        detect_failures = 0
        self.session_state = "REACHING_BASE_HEIGHT"
        print(f"正在到达基础悬停高度：{target_height} cm。")
        if require_reid_target:
            print(f"升高期间持续运行 ReID；目标连续确认 {reacquire_frames} 帧后才允许升高。")

        while monotonic() < deadline and not self.stop_event.is_set():
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
            if require_reid_target or self.display_enabled:
                frame = self._read_frame()
                if frame is None:
                    frame_failures += 1
                    if require_reid_target:
                        target_ready = False
                        target_tracker.reset()
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
                        if require_reid_target:
                            target_ready = target_tracker.observe(result)
                            target_is_fresh = (
                                bool(result.get("found"))
                                and not bool(result.get("is_predicted"))
                                and not bool(result.get("ambiguous"))
                            )
                            if target_is_fresh:
                                target_lost_since = None
                            elif target_lost_since is None:
                                target_lost_since = monotonic()
                        if self.display_enabled:
                            preview = self.detector.draw_debug(frame, result)
                    except Exception as exc:
                        detect_failures += 1
                        if require_reid_target:
                            target_ready = False
                            target_tracker.reset()
                            if target_lost_since is None:
                                target_lost_since = monotonic()
                        print(
                            "升高阶段 ReID 检测异常"
                            f"（{detect_failures}/{self.frame_failure_limit}）：{exc}"
                        )
                        if detect_failures >= self.frame_failure_limit:
                            self.session_state = "FRAME_LOST_LANDING"
                            self._safe_zero_output()
                            return False
                        preview = frame

            if (
                require_reid_target
                and target_lost_since is not None
                and monotonic() - target_lost_since >= target_lost_timeout
            ):
                print("升高阶段目标持续丢失，准备安全降落。")
                self.session_state = "TARGET_LOST_LANDING"
                self._safe_zero_output()
                return False

            height_error = target_height - height
            if not target_ready:
                stable_readings = 0
                command = self.follow_controller.hover()
            elif abs(height_error) <= tolerance:
                stable_readings += 1
                command = self.follow_controller.hover()
            else:
                stable_readings = 0
                command = RCCommand(
                    up_down=vertical_speed if height_error > 0 else -vertical_speed
                )
            self.send_command(command)

            if self.display_enabled:
                import cv2

                if preview is not None:
                    identity_text = "ReID READY" if target_ready else "ReID WAIT - HOVERING"
                    cv2.putText(
                        preview,
                        f"BASE HEIGHT {height}/{target_height} cm | {identity_text}",
                        (20, max(36, preview.shape[0] - 48)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 255) if target_ready else (0, 165, 255),
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
                print(f"基础悬停高度已稳定且目标身份有效：{height} cm。")
                return True
            sleep(self.control_interval)

        self._safe_zero_output()
        if self.stop_event.is_set():
            if self.session_state != "EMERGENCY_STOP":
                self.session_state = "STOPPED"
        else:
            print(f"未能在 {timeout:.1f} 秒内安全到达基础悬停高度，准备降落。")
            self.session_state = "BASE_HEIGHT_FAILED"
        return False

    def _pre_follow_should_abort(self) -> bool:
        """Stop a predefined maneuver after an external or emergency request."""
        return self.stop_event.is_set() or self.emergency_stop

    def _fixed_demo_is_avoiding(self) -> bool:
        """Hold the route timer while obstacle avoidance is overriding it."""
        if self.last_obstacle_result is None:
            return False
        return bool(self.last_obstacle_result.found)

    def _show_pre_follow_progress(self, progress: FixedDemoProgress) -> bool:
        """Display the raw camera during the route and keep q/e responsive."""
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
            "q: stop + land, e: emergency + land",
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

    def send_motion_command(self, command: RCCommand) -> None:
        """Apply the shared obstacle arbiter before sending an autonomous command."""
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

    def apply_obstacle_avoidance(
        self,
        command: RCCommand,
        target_result: Dict[str, object],
        frame: Any,
    ) -> RCCommand:
        """Apply raw obstacle detection only on the no-arbiter fallback path.

        生产装配（app.builder / app.modes）总是提供 motion_arbiter，此时不会走到
        这里。此方法仅为直接传入 obstacle_detector/obstacle_planner 的装配保留，
        与 arbiter 路径保持相同的"目标丢失时不输出避障指令"语义。
        """
        if self.obstacle_detector is None or self.obstacle_planner is None:
            self.last_obstacle_result = None
            self.last_avoidance_decision = None
            return command
        if self.emergency_stop or self.paused or self.stop_event.is_set():
            self.last_obstacle_result = None
            self.last_avoidance_decision = None
            return command
        if self.session_state != "FOLLOWING" or not target_result.get("found"):
            self.last_obstacle_result = None
            self.last_avoidance_decision = None
            return command

        obstacle_result = self.obstacle_detector.detect(frame, target_result)
        decision = self.obstacle_planner.plan(command, obstacle_result)
        self.last_obstacle_result = obstacle_result
        self.last_avoidance_decision = decision
        return decision.command

    def handle_key(self, key: int) -> Optional[str]:
        """Handle q/e and optional p window keys."""
        if self.allow_pause and key == ord("p"):
            self.paused = not self.paused
            self.session_state = "PAUSED" if self.paused else "FOLLOWING"
            self._safe_zero_output()
            if not self.paused and self.motion_arbiter is not None:
                # 恢复时强制下一帧重新检测，避免复用暂停前可能已过期的观测。
                self.motion_arbiter.invalidate_observation()
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

        panel_width = min(frame_width - 24, 520)
        cv2.rectangle(debug_frame, (12, 84), (12 + panel_width, 386), (0, 0, 0), -1)
        target_text = "FOUND" if target_result.get("found") else "LOST"
        battery_text = f"{battery}%" if battery is not None else "N/A"
        height_text = f"{height} cm" if height is not None else "N/A"
        lr, fb, ud, yaw = command.as_tuple()
        debug = self.follow_controller.last_debug
        obstacle_text = "DISABLED"
        obstacle_side = "none"
        obstacle_area = 0.0
        obstacle_reason = ""
        if self.last_obstacle_result is not None:
            obstacle_text = self.last_obstacle_result.state
            obstacle_side = self.last_obstacle_result.side
            obstacle_area = self.last_obstacle_result.area_ratio
        if self.last_avoidance_decision is not None:
            obstacle_text = self.last_avoidance_decision.state
            obstacle_reason = self.last_avoidance_decision.reason
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
            f"OBSTACLE: {obstacle_text} side={obstacle_side} area={obstacle_area:.3f}",
            f"AVOID: {obstacle_reason}",
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
        detect_failures = 0
        stats_started_at = monotonic()
        frame_counter = 0
        command_counter = 0

        while True:
            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in (
                    "LOW_BATTERY_LANDING",
                    "HEIGHT_LIMIT_LANDING",
                    "TARGET_LOST_LANDING",
                    "FRAME_LOST_LANDING",
                ):
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
            try:
                target_result = self.detector.detect(frame)
            except Exception as exc:
                # 检测器异常（如 ReID 推理失败）不中断整个任务：先零输出并重试，
                # 连续失败达到帧数上限后按视频丢失同样的安全策略降落。
                detect_failures += 1
                print(f"目标检测异常（{detect_failures}/{self.frame_failure_limit}）：{exc}")
                self._safe_zero_output()
                if detect_failures >= self.frame_failure_limit:
                    print("检测器连续异常，准备安全降落。")
                    self.session_state = "FRAME_LOST_LANDING"
                    break
                sleep(0.05)
                continue
            detect_failures = 0
            command, lost_action = self.process_detection(target_result, frame_width, frame_height)
            if self.motion_arbiter is not None and not (self.paused or self.emergency_stop):
                decision = self.motion_arbiter.decide(
                    desired_command=command,
                    frame=frame,
                    context=MotionContext(mode=self.mode_label, target_result=target_result),
                )
                self.last_obstacle_result = decision.observation
                self.last_avoidance_decision = decision
                command = decision.command
                if decision.requires_landing:
                    self.session_state = "OBSTACLE_FAILSAFE_LANDING"
                    self._safe_zero_output()
                    break
            elif self.motion_arbiter is None:
                # 仅当没有装配 motion_arbiter 时才使用原始检测器/规划器回退路径。
                command = self.apply_obstacle_avoidance(command, target_result, frame)
            # 暂停/急停时冻结自主运动：不运行仲裁（既不覆盖指令，也不会在暂停中
            # 被避障超时强制降落），command 保持 process_detection 返回的悬停。

            battery = self._read_battery()
            height = self._read_height()
            if self.stop_event.is_set():
                if self.emergency_stop:
                    self.session_state = "EMERGENCY_STOP"
                elif self.session_state not in (
                    "LOW_BATTERY_LANDING",
                    "HEIGHT_LIMIT_LANDING",
                    "TARGET_LOST_LANDING",
                    "FRAME_LOST_LANDING",
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
                    f"飞行高度 {height} cm 超出安全范围 "
                    f"[{self.safety_manager.config.min_height_cm}, "
                    f"{self.safety_manager.config.max_height_cm}] cm，准备安全降落。"
                )
                self.session_state = "HEIGHT_LIMIT_LANDING"
                self._safe_zero_output()
                break

            if lost_action == "land":
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

    def _reset_tracking_state(self) -> None:
        """Clear detector and controller state before every independent follow task."""
        reset_method = getattr(self.detector, "reset", None)
        if callable(reset_method):
            reset_method()
        self.follow_controller.reset()
        if self.motion_arbiter is not None:
            if not self.motion_arbiter.is_active:
                self.motion_arbiter.reset(self.mode_label)
        else:
            obstacle_reset_method = getattr(self.obstacle_detector, "reset", None)
            if callable(obstacle_reset_method):
                obstacle_reset_method()
            if self.obstacle_planner is not None:
                self.obstacle_planner.reset()
        self.last_obstacle_result = None
        self.last_avoidance_decision = None

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
