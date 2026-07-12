"""Tests for fake camera frames and Agent visual follow session logic."""

import inspect
import sys
import unittest
from pathlib import Path
from time import monotonic


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from agent.follow_session import AgentFollowSession
from agent.tools import AgentTools
from control.follow_control import FollowController, RCCommand
from control.follow_session import FollowSession
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
import main
from vision.target_detect import TargetDetector


def build_safety() -> SafetyManager:
    """Build a test safety manager with short target-loss timing."""
    return SafetyManager(
        SafetyConfig(
            min_battery_takeoff=30,
            low_battery_land=20,
            max_height_cm=150,
            min_height_cm=60,
            max_rc_speed=25,
            target_lost_hover_seconds=1,
            target_lost_land_seconds=2,
        )
    )


def build_session(drone: FakeDroneAdapter, safety: SafetyManager) -> AgentFollowSession:
    """Create a no-GUI Agent follow session for unit tests."""
    return AgentFollowSession(
        drone=drone,
        safety_manager=safety,
        detector=TargetDetector(),
        follow_controller=FollowController(safety_manager=safety),
        config={
            "display_agent_camera": False,
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

    def test_large_horizontal_error_suppresses_forward(self) -> None:
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

        self.assertLess(large_error_far.forward_backward, centered_far.forward_backward)

    def test_small_horizontal_error_allows_forward(self) -> None:
        controller = self.build_controller()
        command = controller.compute_command(
            {"found": True, "center": (340, 240), "area": 1000, "bbox": (325, 225, 30, 30)},
            640,
            480,
        )

        self.assertGreater(command.forward_backward, 0)

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


class AgentFollowSessionTestCase(unittest.TestCase):
    """Verify Agent follow session safety-state behavior without GUI."""

    def test_follow_and_agent_use_shared_follow_session(self) -> None:
        follow_source = inspect.getsource(main.run_follow)
        agent_source = inspect.getsource(AgentTools.start_follow_task)

        self.assertIn("FollowSession", follow_source)
        self.assertIn("FollowSession", agent_source)
        self.assertTrue(issubclass(AgentFollowSession, FollowSession))

    def test_pause_forces_zero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = build_session(drone, safety)

        session.handle_key(ord("p"))
        session.send_command(RCCommand(12, 12, 0, 0))

        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.agent_state, "PAUSED")

    def test_emergency_stop_blocks_nonzero_rc(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = build_session(drone, safety)

        action = session.handle_key(ord("e"))
        session.send_command(RCCommand(12, 12, 0, 0))

        self.assertEqual(action, "emergency")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.agent_state, "EMERGENCY_STOP")

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
        self.assertEqual(session.agent_state, "TARGET_LOST_LANDING")

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

        self.assertEqual(session.agent_state, "FRAME_LOST_LANDING")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_exception_cleanup_sends_zero_and_lands(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = build_safety()
        session = RaisingLoopSession(
            drone=drone,
            safety_manager=safety,
            detector=TargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_agent_camera": False,
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
