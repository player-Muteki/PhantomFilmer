"""Tests that follow sessions reset optional detector state once per task."""

import unittest
from unittest.mock import patch

from control.follow_control import FollowController
from control.follow_session import FollowSession
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager


class RecordingDetector:
    def __init__(self) -> None:
        self.reset_calls = 0
        self.detect_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1

    def detect(self, frame):
        self.detect_calls += 1
        return {"found": False, "center": None, "area": 0.0, "bbox": None}

    def draw_debug(self, frame, result):
        return frame


class NoResetDetector:
    def detect(self, frame):
        return {"found": False, "center": None, "area": 0.0, "bbox": None}

    def draw_debug(self, frame, result):
        return frame


class FailingPrepareDetector(NoResetDetector):
    def prepare(self) -> None:
        raise RuntimeError("model preflight failed")


class ThreeFrameSession(FollowSession):
    def _start_camera(self) -> None:
        self.streaming = True

    def _loop(self) -> None:
        for _ in range(3):
            self.detector.detect(None)


class RecordingFollowController(FollowController):
    def __init__(self, safety_manager) -> None:
        super().__init__(safety_manager=safety_manager)
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1
        super().reset()


class RecordingObstacleDetector:
    def __init__(self) -> None:
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1

    def detect(self, frame, target_result):
        return None

    def draw_debug(self, frame, result):
        return frame


class RecordingObstaclePlanner:
    def __init__(self) -> None:
        self.reset_calls = 0

    def reset(self) -> None:
        self.reset_calls += 1


class FollowSessionDetectorResetTestCase(unittest.TestCase):
    def build_session(self, detector):
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        return ThreeFrameSession(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )

    def test_reset_is_called_once_before_a_new_session(self) -> None:
        detector = RecordingDetector()
        with patch("control.follow_session.sleep", return_value=None):
            self.build_session(detector).run()
        self.assertEqual(detector.reset_calls, 1)
        self.assertEqual(detector.detect_calls, 3)

    def test_detector_without_reset_remains_supported(self) -> None:
        with patch("control.follow_session.sleep", return_value=None):
            self.build_session(NoResetDetector()).run()

    def test_detector_prepare_failure_happens_before_takeoff(self) -> None:
        session = self.build_session(FailingPrepareDetector())
        with self.assertRaisesRegex(RuntimeError, "model preflight failed"):
            session.run()
        self.assertEqual(session.drone.height_cm, 0)
        self.assertFalse(session.airborne)
        self.assertFalse(session.streaming)

    def test_controller_reset_is_called_once_before_a_new_session(self) -> None:
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        controller = RecordingFollowController(safety)
        session = ThreeFrameSession(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=NoResetDetector(),
            follow_controller=controller,
            config={"display_console_camera": False},
            mode_label="FAKE",
        )
        with patch("control.follow_session.sleep", return_value=None):
            session.run()
        self.assertEqual(controller.reset_calls, 1)

    def test_obstacle_modules_reset_once_before_a_new_session(self) -> None:
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        obstacle_detector = RecordingObstacleDetector()
        obstacle_planner = RecordingObstaclePlanner()
        session = ThreeFrameSession(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=NoResetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
            obstacle_detector=obstacle_detector,
            obstacle_planner=obstacle_planner,
        )
        with patch("control.follow_session.sleep", return_value=None):
            session.run()
        self.assertEqual(obstacle_detector.reset_calls, 1)
        self.assertEqual(obstacle_planner.reset_calls, 1)


if __name__ == "__main__":
    unittest.main()
