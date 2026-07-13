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


class ThreeFrameSession(FollowSession):
    def _start_camera(self) -> None:
        self.streaming = True

    def _loop(self) -> None:
        for _ in range(3):
            self.detector.detect(None)


class FollowSessionDetectorResetTestCase(unittest.TestCase):
    def build_session(self, detector):
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        return ThreeFrameSession(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=detector,
            follow_controller=FollowController(safety_manager=safety),
            config={"display_agent_camera": False},
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


if __name__ == "__main__":
    unittest.main()
