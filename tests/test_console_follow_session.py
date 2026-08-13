"""Tests for fake camera frames and Console visual follow session logic."""

import inspect
import sys
import unittest
from unittest.mock import patch
from pathlib import Path
from threading import Event
from time import monotonic, sleep


try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    cv2 = None


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

import console.tools as console_tools_module
from console.follow_session import ConsoleFollowSession
from console.tools import ConsoleTools
from control.follow_control import FollowController, RCCommand
from control.follow_session import FollowSession
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
import main
from app import modes as app_modes
from vision.aruco_detect import ArucoTargetDetector
from vision.target_detect import TargetDetector


def build_safety(max_rc_speed: int = 25) -> SafetyManager:
    """Build a test safety manager with short target-loss timing."""
    return SafetyManager(
        SafetyConfig(
            min_battery_takeoff=30,
            low_battery_land=20,
            max_height_cm=150,
            min_height_cm=60,
            max_rc_speed=max_rc_speed,
            target_lost_hover_seconds=1,
            target_lost_land_seconds=2,
        )
    )


def build_session(drone: FakeDroneAdapter, safety: SafetyManager) -> ConsoleFollowSession:
    """Create a no-GUI Console follow session for unit tests."""
    return ConsoleFollowSession(
        drone=drone,
        safety_manager=safety,
        detector=TargetDetector(),
        follow_controller=FollowController(safety_manager=safety),
        config={
            "display_console_camera": False,
            "camera_width": 640,
            "camera_height": 480,
            "frame_failure_limit": 3,
        },
        mode_label="FAKE",
    )


class RaisingLoopSession(FollowSession):
    """Session that fails inside the follow loop for cleanup testing."""

    def _loop(self) -> None:
        raise RuntimeError("forced loop failure")


class FailedHeightTakeoffSession(FollowSession):
    """Session that reports no takeoff height after a successful command."""

    def _read_height(self):
        return 0


class ClimbingFakeDrone(FakeDroneAdapter):
    """Apply vertical RC commands to synthetic height telemetry."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.rc_history = []

    def move_rc(self, left_right: int, forward_backward: int, up_down: int, yaw: int) -> None:
        self.rc_history.append((left_right, forward_backward, up_down, yaw))
        super().move_rc(left_right, forward_backward, up_down, yaw)
        if self.height_cm > 0 and up_down:
            step = max(1, abs(int(up_down)) // 4)
            self.height_cm += step if up_down > 0 else -step


class PhaseSequenceDetector:
    """Return a target-presence sequence and record phase inference calls."""

    def __init__(self, found_sequence) -> None:
        self.found_sequence = list(found_sequence)
        self.detect_calls = 0

    def detect(self, frame):
        index = min(self.detect_calls, len(self.found_sequence) - 1)
        found = bool(self.found_sequence[index])
        self.detect_calls += 1
        return {
            "found": found,
            "is_predicted": False,
            "ambiguous": False,
            "center": (320, 240) if found else None,
            "area": 12000 if found else 0,
            "bbox": (280, 120, 80, 240) if found else None,
        }

    def draw_debug(self, frame, result):
        return frame


class FakePhaseCamera:
    def __init__(self, drone) -> None:
        self.drone = drone

    def read_frame(self):
        return self.drone.get_frame()


class ImmediateSession:
    """Session stub used to verify Console starts follow in the background."""

    run_count = 0

    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs

    def run(self):
        ImmediateSession.run_count += 1
        return type("Result", (), {"state": "STOPPED", "airborne": False, "streaming": False})()


class BlockingSession:
    """Session stub that remains active until the console requests a stop."""

    started = Event()

    def __init__(self, **kwargs) -> None:
        self.stop_event = kwargs["stop_event"]
        self.airborne = True

    def run(self):
        BlockingSession.started.set()
        self.stop_event.wait(timeout=2)
        self.airborne = False
        return type("Result", (), {"state": "STOPPED", "airborne": False, "streaming": False})()

    def request_stop(self) -> None:
        self.stop_event.set()

    def request_emergency_stop(self) -> None:
        self.stop_event.set()


class StuckSession:
    """Session stub used to verify timeout landing fallback."""

    airborne = False


@unittest.skipIf(cv2 is None, "opencv-contrib-python is required for fake camera and visual detection tests")
class FakeAdapterTestCase(unittest.TestCase):
    """Verify the fake camera behaves like a real image source."""

    def test_fake_frame_shape_and_detection(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        frame = drone.get_frame()
        detector = TargetDetector()
        result = detector.detect(frame)

        self.assertEqual(frame.shape, (480, 640, 3))
        self.assertTrue(result["found"])

    def test_fake_target_center_and_area_change(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False, target_speed=8)
        detector = TargetDetector()
        centers = []
        areas = []

        for _ in range(35):
            result = detector.detect(drone.get_frame())
            if result["found"]:
                centers.append(result["center"])
                areas.append(round(float(result["area"] or 0.0), 1))

        self.assertGreater(len(set(centers)), 1)
        self.assertGreater(len(set(areas)), 1)

    def test_fake_target_lost_and_rc_memory(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.force_target_visible = False
        detector = TargetDetector()

        result = detector.detect(drone.get_frame())
        drone.move_rc(3, 4, 0, -2)

        self.assertFalse(result["found"])
        self.assertEqual(drone.last_rc_command, (3, 4, 0, -2))

    @unittest.skipUnless(
        cv2 is not None and hasattr(cv2, "aruco"),
        "OpenCV ArUco support is required",
    )
    def test_fake_aruco_frame_is_detected(self) -> None:
        config = {
            "vision": {
                "detector_type": "aruco",
                "aruco_dictionary": "DICT_4X4_50",
                "target_marker_id": 23,
                "min_marker_area": 100,
            }
        }
        drone = FakeDroneAdapter(
            verbose_rc=False,
            detector_type="aruco",
            aruco_dictionary="DICT_4X4_50",
            target_marker_id=23,
        )
        detector = ArucoTargetDetector.from_config(config)

        result = detector.detect(drone.get_frame())

        self.assertTrue(result["found"])
        self.assertEqual(result["marker_id"], 23)


class FollowControllerTestCase(unittest.TestCase):
    """Verify yaw-first and area-ratio follow control."""

    def build_controller(self, max_rc_speed: int = 25) -> FollowController:
        safety = SafetyManager(
            SafetyConfig(
                min_battery_takeoff=30,
                low_battery_land=20,
                max_height_cm=150,
                min_height_cm=60,
                max_rc_speed=max_rc_speed,
                target_lost_hover_seconds=1,
                target_lost_land_seconds=2,
            )
        )
        return FollowController.from_config(
            safety_manager=safety,
            config={
                "horizontal_dead_zone_ratio": 0.08,
                "yaw_kp": 80,
                "minimum_yaw_speed": 10,
                "maximum_yaw_speed": 25,
                "target_area_ratio_min": 0.015,
                "target_area_ratio_max": 0.06,
                "forward_kp": 500,
                "minimum_forward_speed": 10,
                "maximum_forward_speed": 20,
                "large_horizontal_error_ratio": 0.28,
                "forward_speed_while_turning_ratio": 0.25,
                "vertical_dead_zone_ratio": 0.10,
                "vertical_speed": 8,
            },
        )

    def test_left_target_uses_left_yaw_not_left_right(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (120, 240), "area": 10000, "bbox": (90, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.left_right, 0)
        self.assertLess(command.yaw, 0)

    def test_right_target_uses_right_yaw_not_left_right(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (520, 240), "area": 10000, "bbox": (490, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.left_right, 0)
        self.assertGreater(command.yaw, 0)

    def test_center_dead_zone_has_zero_yaw(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (330, 240), "area": 10000, "bbox": (300, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.yaw, 0)

    def test_far_target_moves_forward(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 1000, "bbox": (305, 225, 30, 30)},
            640,
            480,
        )

        self.assertGreater(command.forward_backward, 0)

    def test_suitable_distance_has_zero_forward_back(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 10000, "bbox": (290, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.forward_backward, 0)

    def test_near_target_moves_backward(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 30000, "bbox": (240, 160, 160, 160)},
            640,
            480,
        )

        self.assertLess(command.forward_backward, 0)

    def test_distance_control_uses_full_speed_outside_hover_band(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(max_rc_speed=35),
            config={
                "target_area_ratio_min": 0.030,
                "target_area_ratio_max": 0.080,
                "minimum_forward_speed": 12,
                "maximum_forward_speed": 35,
            },
        )

        far = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 0.020 * 640 * 480}, 640, 480
        )
        close = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 0.090 * 640 * 480}, 640, 480
        )

        self.assertEqual(far.forward_backward, 35)
        self.assertEqual(close.forward_backward, -35)

    def test_horizontal_error_allows_alignment_speed_forward_while_turning(self) -> None:
        controller = self.build_controller()
        centered_far = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 1000, "bbox": (305, 225, 30, 30)},
            640,
            480,
        )
        large_error_far = controller.compute_command(
            {"found": True, "center": (40, 240), "area": 1000, "bbox": (25, 225, 30, 30)},
            640,
            480,
        )

        self.assertGreater(centered_far.forward_backward, 0)
        self.assertEqual(large_error_far.forward_backward, controller.forward_speed_while_aligning)
        self.assertLess(large_error_far.yaw, 0)

    def test_small_horizontal_error_allows_forward(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (340, 240), "area": 1000, "bbox": (325, 225, 30, 30)},
            640,
            480,
        )

        self.assertGreater(command.forward_backward, 0)

    def test_vertical_error_allows_alignment_speed_forward(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 100), "area": 1000, "bbox": (305, 85, 30, 30)},
            640,
            480,
        )

        self.assertGreater(command.up_down, 0)
        self.assertEqual(command.forward_backward, controller.forward_speed_while_aligning)

    def test_yaw_and_vertical_adjustments_can_run_together(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (120, 100), "area": 10000, "bbox": (90, 70, 60, 60)},
            640,
            480,
        )

        self.assertLess(command.yaw, 0)
        self.assertGreater(command.up_down, 0)
        self.assertEqual(command.forward_backward, 0)

    def test_three_axis_alignment_uses_fixed_forward_speed(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (120, 100), "area": 1000, "bbox": (105, 85, 30, 30)},
            640,
            480,
        )

        self.assertLess(command.yaw, 0)
        self.assertGreater(command.up_down, 0)
        self.assertEqual(command.forward_backward, controller.forward_speed_while_aligning)

    def test_configured_vertical_and_alignment_speeds_are_applied(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(max_rc_speed=35),
            config={
                "vertical_speed": 20,
                "forward_speed_while_aligning": 16,
                "target_area_ratio_min": 0.03,
                "target_area_ratio_max": 0.08,
            },
        )
        command = controller.compute_command(
            {"found": True, "center": (120, 100), "area": 1000}, 640, 480
        )

        self.assertEqual(command.up_down, 20)
        self.assertEqual(command.forward_backward, 16)

    def test_lock_hovers_after_target_is_stable_for_configured_frames(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(),
            config={
                "target_area_ratio_min": 0.02,
                "target_area_ratio_max": 0.04,
                "target_lock_stable_frames": 2,
                "target_lock_exit_area_ratio_min": 0.015,
                "target_lock_exit_area_ratio_max": 0.05,
                "target_lock_exit_horizontal_dead_zone_ratio": 0.12,
                "target_lock_exit_vertical_dead_zone_ratio": 0.15,
            },
        )
        result = {"found": True, "is_predicted": False, "center": (320, 240), "area": 9000}

        controller.compute_command(result, 640, 480)
        command = controller.compute_command(result, 640, 480)

        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(controller.last_debug.target_state, "LOCKED")

    def test_locked_target_ignores_small_motion_but_resumes_following_when_moved(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(),
            config={
                "target_area_ratio_min": 0.02,
                "target_area_ratio_max": 0.04,
                "target_lock_stable_frames": 1,
                "target_lock_exit_area_ratio_min": 0.015,
                "target_lock_exit_area_ratio_max": 0.05,
                "target_lock_exit_horizontal_dead_zone_ratio": 0.12,
                "target_lock_exit_vertical_dead_zone_ratio": 0.15,
            },
        )
        stable = {"found": True, "is_predicted": False, "center": (320, 240), "area": 9000}
        controller.compute_command(stable, 640, 480)

        small_motion = controller.compute_command(
            {"found": True, "is_predicted": False, "center": (350, 240), "area": 9000}, 640, 480
        )
        moved = controller.compute_command(
            {"found": True, "is_predicted": False, "center": (320, 240), "area": 3000}, 640, 480
        )

        self.assertEqual(small_motion.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(controller.last_debug.target_state, "FOUND")
        self.assertGreater(moved.forward_backward, 0)

    def test_predicted_detection_cannot_enter_lock(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(),
            config={"target_area_ratio_min": 0.02, "target_area_ratio_max": 0.04, "target_lock_stable_frames": 1},
        )
        command = controller.compute_command(
            {"found": True, "is_predicted": True, "center": (320, 240), "area": 9000}, 640, 480
        )

        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(controller.last_debug.target_state, "FOUND")

    def test_predicted_detection_releases_existing_lock(self) -> None:
        controller = FollowController.from_config(
            safety_manager=build_safety(),
            config={"target_area_ratio_min": 0.02, "target_area_ratio_max": 0.04, "target_lock_stable_frames": 1},
        )
        controller.compute_command(
            {"found": True, "is_predicted": False, "center": (320, 240), "area": 9000}, 640, 480
        )
        controller.compute_command(
            {"found": True, "is_predicted": True, "center": (320, 240), "area": 9000}, 640, 480
        )

        self.assertEqual(controller.last_debug.target_state, "FOUND")

    def test_lost_target_outputs_zero(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": False, "center": None, "area": 0, "bbox": None},
            640,
            480,
        )

        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))

    def test_upper_target_moves_up(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 100), "area": 10000, "bbox": (290, 70, 60, 60)},
            640,
            480,
        )

        self.assertGreater(command.up_down, 0)

    def test_lower_target_moves_down(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 380), "area": 10000, "bbox": (290, 350, 60, 60)},
            640,
            480,
        )

        self.assertLess(command.up_down, 0)

    def test_vertical_dead_zone_has_zero_up_down(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (320, 240), "area": 10000, "bbox": (290, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.up_down, 0)

    def test_vertical_speed_is_safety_limited(self) -> None:
        safety = SafetyManager(
            SafetyConfig(
                min_battery_takeoff=30,
                low_battery_land=20,
                max_height_cm=150,
                min_height_cm=60,
                max_rc_speed=5,
                target_lost_hover_seconds=1,
                target_lost_land_seconds=2,
            )
        )
        controller = FollowController.from_config(
            safety_manager=safety,
            config={"vertical_speed": 30, "vertical_dead_zone_ratio": 0.10},
        )
        command = controller.compute_command(
            {"found": True, "center": (320, 100), "area": 10000, "bbox": (290, 70, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.up_down, 5)

    def test_safety_limit_clamps_all_outputs(self) -> None:
        controller = self.build_controller(max_rc_speed=12)
        command = controller.compute_command(
            {"found": True, "center": (640, 0), "area": 1, "bbox": (630, 0, 5, 5)},
            640,
            480,
        )

        self.assertTrue(all(abs(value) <= 12 for value in command.as_tuple()))

    def test_missing_config_fields_use_defaults(self) -> None:
        safety = build_safety()
        controller = FollowController.from_config(safety_manager=safety, config={})

        command = controller.compute_command(
            {"found": True, "center": (520, 240), "area": 10000, "bbox": (490, 210, 60, 60)},
            640,
            480,
        )

        self.assertEqual(command.left_right, 0)
        self.assertGreater(command.yaw, 0)


class ConsoleFollowSessionTestCase(unittest.TestCase):
    """Verify Console follow session safety-state behavior without GUI."""

    def test_follow_and_console_use_shared_follow_session(self) -> None:
        follow_source = inspect.getsource(app_modes.run_follow)
        console_source = inspect.getsource(ConsoleTools.start_follow_task)

        self.assertIn("FollowSession", follow_source)
        self.assertIn("FollowSession", console_source)
        self.assertTrue(issubclass(ConsoleFollowSession, FollowSession))

    def test_console_start_follow_task_runs_session_in_background(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.connect()
        safety = build_safety()
        tools = ConsoleTools(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )
        tools.connected = True
        ImmediateSession.run_count = 0

        original_session = console_tools_module.FollowSession
        console_tools_module.FollowSession = ImmediateSession
        try:
            result = tools.start_follow_task()
            stopped = tools.wait_for_task(timeout=1)
        finally:
            console_tools_module.FollowSession = original_session

        self.assertTrue(result)
        self.assertTrue(stopped)
        self.assertEqual(ImmediateSession.run_count, 1)
        self.assertFalse(tools.is_task_active())
        self.assertEqual(tools.current_mode, "STOPPED")

    def test_console_can_stop_background_follow_task(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.connect()
        safety = build_safety()
        tools = ConsoleTools(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )
        tools.connected = True
        BlockingSession.started.clear()

        original_session = console_tools_module.FollowSession
        console_tools_module.FollowSession = BlockingSession
        try:
            self.assertTrue(tools.start_follow_task())
            self.assertTrue(BlockingSession.started.wait(timeout=1))
            self.assertTrue(tools.is_task_active())
            tools.stop_task()
        finally:
            console_tools_module.FollowSession = original_session

        self.assertFalse(tools.is_task_active())
        self.assertEqual(tools.current_mode, "待机")

    def test_shutdown_timeout_attempts_landing_without_airborne_flag(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.connect()
        drone.height_cm = 70
        safety = build_safety()
        tools = ConsoleTools(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"task_stop_timeout_seconds": 1},
            mode_label="FAKE",
        )
        tools.connected = True
        tools.wait_for_task = lambda timeout=None: False

        tools._wait_for_session_shutdown(StuckSession())

        self.assertEqual(drone.get_height(), 0)

    def test_pause_forces_zero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = build_session(drone, safety)

        session.handle_key(ord("p"))
        session.send_command(RCCommand(12, 12, 0, 0))

        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.console_state, "PAUSED")

    def test_emergency_stop_blocks_nonzero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = build_session(drone, safety)

        action = session.handle_key(ord("e"))
        session.send_command(RCCommand(12, 12, 0, 0))

        self.assertEqual(action, "emergency")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.console_state, "EMERGENCY_STOP")

    def test_long_target_loss_enters_landing_state(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        safety._target_lost_since = monotonic() - 3.0
        session = build_session(drone, safety)

        command, action = session.process_detection(
            {"found": False, "center": None, "area": 0, "bbox": None},
            frame_width=640,
            frame_height=480,
        )

        self.assertEqual(action, "land")
        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(session.console_state, "TARGET_LOST_LANDING")

    def test_reid_direct_takeoff_can_search_without_ground_lock(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 1, 2))
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "target_search": {"enabled": True, "hold_seconds": 1.0},
            },
            mode_label="REID DIRECT TEST",
            initial_target_lock_frames=0,
            enable_target_search=True,
        )

        command, action = session.process_detection(
            {"found": False, "center": None, "area": 0, "bbox": None},
            frame_width=640,
            frame_height=480,
            height_cm=150,
            now=0.0,
        )

        self.assertTrue(session.target_search_enabled)
        self.assertEqual(action, "search")
        self.assertEqual(command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(session.console_state, "LOST_HOLD")

    def test_frame_failure_stops_with_zero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = build_session(drone, safety)
        session.frame_failure_limit = 1
        session.camera = type(
            "NullCamera",
            (),
            {
                "read_frame": lambda self: None,
                "stop": lambda self: None,
            },
        )()
        session.streaming = True

        session._loop()

        self.assertEqual(session.console_state, "FRAME_LOST_LANDING")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_height_outside_safe_range_stops_with_zero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.height_cm = 151
        safety = build_safety()
        session = build_session(drone, safety)
        session.camera = type(
            "SingleFrameCamera",
            (),
            {
                "read_frame": lambda self: type(
                    "Frame",
                    (),
                    {"shape": (480, 640, 3)},
                )(),
                "stop": lambda self: None,
            },
        )()
        session.detector = type(
            "Detector",
            (),
            {
                "detect": lambda self, frame: {
                    "found": True,
                    "center": (320, 240),
                    "area": 12000,
                    "bbox": (280, 200, 80, 80),
                },
            },
        )()

        session._loop()

        self.assertEqual(session.console_state, "HEIGHT_LIMIT_LANDING")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_low_positive_ground_clearance_does_not_force_landing(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.height_cm = 20
        safety = build_safety()
        session = build_session(drone, safety)
        session.stop_event.set()

        self.assertTrue(safety.check_height(drone.height_cm))
        self.assertNotEqual(session.console_state, "HEIGHT_LIMIT_LANDING")

    def test_reaches_configured_base_hover_height_before_follow(self) -> None:
        drone = ClimbingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        safety = SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 1, 2))
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "control_interval": 0.02,
                "base_hover_height_cm": 180,
                "base_hover_height_tolerance_cm": 5,
                "base_hover_vertical_speed": 20,
                "base_hover_stable_readings": 2,
            },
            mode_label="FAKE",
        )

        self.assertTrue(session._reach_base_hover_height())
        self.assertGreaterEqual(drone.height_cm, 175)
        self.assertLessEqual(drone.height_cm, 185)
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.console_state, "BASE_HEIGHT_READY")

    def test_reid_continues_during_takeoff_stabilization(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        detector = PhaseSequenceDetector([True])
        safety = SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 1, 2))
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False, "control_interval": 0.02},
            mode_label="REID TEST",
            initial_target_lock_frames=1,
        )
        session.camera = FakePhaseCamera(drone)
        clock = [0.0]

        def advance_clock():
            clock[0] += 0.05
            return clock[0]

        with patch("control.follow_session.monotonic", side_effect=advance_clock), patch(
            "control.follow_session.sleep", return_value=None
        ):
            self.assertTrue(session._wait_for_takeoff_stabilization(0.2))

        self.assertGreater(detector.detect_calls, 0)

    def test_base_height_keeps_climbing_while_reid_is_reacquired(self) -> None:
        drone = ClimbingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        detector = PhaseSequenceDetector([False, False, True, True, True])
        safety = SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 1, 2))
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "control_interval": 0.02,
                "base_hover_height_cm": 180,
                "base_hover_vertical_speed": 20,
                "base_hover_stable_readings": 2,
            },
            mode_label="REID TEST",
            initial_target_lock_frames=1,
        )
        session.camera = FakePhaseCamera(drone)

        self.assertTrue(session._reach_base_hover_height())
        first_climb_index = next(
            index for index, command in enumerate(drone.rc_history) if command[2] > 0
        )
        self.assertEqual(first_climb_index, 0)
        self.assertGreaterEqual(detector.detect_calls, 5)
        self.assertGreaterEqual(drone.height_cm, 175)
        self.assertEqual(session.console_state, "BASE_HEIGHT_READY")

    def test_base_height_climbs_without_reid_target(self) -> None:
        drone = ClimbingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        detector = PhaseSequenceDetector([False])
        safety = SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 1, 2))
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "control_interval": 0.02,
                "base_hover_height_cm": 180,
            },
            mode_label="REID TEST",
            initial_target_lock_frames=1,
        )
        session.camera = FakePhaseCamera(drone)
        self.assertTrue(session._reach_base_hover_height())

        self.assertTrue(any(command[2] > 0 for command in drone.rc_history))
        self.assertGreaterEqual(drone.height_cm, 175)
        self.assertEqual(session.console_state, "BASE_HEIGHT_READY")

    def test_takeoff_height_ignores_transient_zero_readings(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "takeoff_height_verify_timeout_seconds": 1,
                "takeoff_height_sample_interval_seconds": 0.05,
                "takeoff_height_min_cm": 20,
                "takeoff_height_stable_readings": 3,
            },
            mode_label="FAKE",
        )
        readings = iter([0, 5, 22, 25, 28, 30])
        session._read_height = lambda: next(readings)

        with patch("control.follow_session.sleep", return_value=None):
            self.assertTrue(session._verify_takeoff_height())

        self.assertEqual(session.session_state, "TAKEOFF_HEIGHT_READY")

    def test_takeoff_height_sampling_does_not_wait_for_reid(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        detector = PhaseSequenceDetector([True])
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "takeoff_height_verify_timeout_seconds": 1,
                "takeoff_height_sample_interval_seconds": 0.05,
                "takeoff_height_min_cm": 20,
                "takeoff_height_stable_readings": 3,
            },
            mode_label="REID TEST",
            initial_target_lock_frames=1,
        )
        readings = iter([30, 32, 31])
        session._read_height = lambda: next(readings)

        with patch("control.follow_session.sleep", return_value=None):
            self.assertTrue(session._verify_takeoff_height())

        self.assertEqual(detector.detect_calls, 0)

    def test_ground_clearance_uses_five_sample_median_and_rejects_negative(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "height_filter_window": 5,
                "height_max_valid_cm": 500,
            },
            mode_label="FAKE",
        )
        readings = iter([100, 102, 500, 101, 99, -20])
        drone.get_height = lambda: next(readings)

        self.assertEqual([session._read_height() for _ in range(5)], [100, 101, 102, 102, 101])
        self.assertIsNone(session._read_height())
        self.assertEqual(session.last_raw_height, -20)

    def test_rejects_base_hover_height_outside_safety_range(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        drone.height_cm = 70
        safety = build_safety()
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False, "base_hover_height_cm": 180},
            mode_label="FAKE",
        )

        self.assertFalse(session._reach_base_hover_height())
        self.assertEqual(session.console_state, "BASE_HEIGHT_FAILED")

    def test_exception_cleanup_sends_zero_and_lands(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = RaisingLoopSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "camera_width": 640,
                "camera_height": 480,
            },
            mode_label="FAKE",
        )

        with self.assertRaises(RuntimeError):
            session.run()

        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(drone.get_height(), 0)
        self.assertFalse(session.streaming)

    def test_failed_takeoff_height_still_attempts_landing(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = FailedHeightTakeoffSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "takeoff_height_verify_enabled": True,
                "takeoff_height_verify_timeout_seconds": 0.2,
                "takeoff_height_sample_interval_seconds": 0.05,
            },
            mode_label="FAKE",
        )

        result = session.run()

        self.assertEqual(result.state, "STOPPED")
        self.assertEqual(drone.get_height(), 0)
        self.assertFalse(result.airborne)

    def test_djitellopy_only_imported_by_tello_adapter(self) -> None:
        project_root = PROJECT_ROOT
        offenders = []
        import_needle = "import " + "djitellopy"
        from_needle = "from " + "djitellopy"
        for path in project_root.rglob("*.py"):
            if path.name == "tello_adapter.py":
                continue
            text = path.read_text(encoding="utf-8")
            if import_needle in text or from_needle in text:
                offenders.append(str(path.relative_to(project_root)))

        self.assertEqual(offenders, [])


if __name__ == "__main__":
    unittest.main()
