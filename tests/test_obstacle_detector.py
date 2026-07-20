"""Tests for visual obstacle-risk detection."""

import unittest

import numpy as np

try:
    import cv2  # noqa: F401
except ModuleNotFoundError:
    cv2 = None

from vision.obstacle_detect import ObstacleDetector


@unittest.skipIf(cv2 is None, "OpenCV is not installed")
class ObstacleDetectorTestCase(unittest.TestCase):
    def build_detector(self) -> ObstacleDetector:
        return ObstacleDetector(
            risk_zone_width_ratio=0.6,
            risk_zone_height_ratio=0.6,
            minimum_obstacle_area=200,
            caution_area_ratio=0.01,
            blocked_area_ratio=0.08,
        )

    def frame(self) -> np.ndarray:
        return np.zeros((200, 300, 3), dtype=np.uint8)

    def test_from_config_uses_obstacle_block(self) -> None:
        detector = ObstacleDetector.from_config({"obstacle": {"minimum_obstacle_area": 321}})
        self.assertEqual(detector.minimum_obstacle_area, 321)

    def test_invalid_frame_returns_clear(self) -> None:
        result = self.build_detector().detect(None, {})
        self.assertFalse(result.found)
        self.assertEqual(result.state, "CLEAR")

    def test_large_object_inside_risk_zone_is_blocked(self) -> None:
        frame = self.frame()
        frame[60:160, 110:210] = 255
        result = self.build_detector().detect(frame, {"found": False, "bbox": None})
        self.assertTrue(result.found)
        self.assertEqual(result.state, "BLOCKED")
        self.assertEqual(result.side, "center")

    def test_small_object_below_threshold_is_clear(self) -> None:
        frame = self.frame()
        frame[95:105, 145:155] = 255
        result = self.build_detector().detect(frame, {"found": False, "bbox": None})
        self.assertFalse(result.found)

    def test_object_outside_risk_zone_is_ignored(self) -> None:
        frame = self.frame()
        frame[10:60, 10:60] = 255
        result = self.build_detector().detect(frame, {"found": False, "bbox": None})
        self.assertFalse(result.found)

    def test_target_bbox_is_excluded(self) -> None:
        frame = self.frame()
        frame[60:160, 110:210] = 255
        result = self.build_detector().detect(
            frame,
            {"found": True, "bbox": (105, 55, 110, 110)},
        )
        self.assertFalse(result.found)

    def test_left_and_right_sides_are_reported(self) -> None:
        detector = self.build_detector()
        left_frame = self.frame()
        left_frame[60:160, 60:115] = 255
        left = detector.detect(left_frame, {"found": False, "bbox": None})
        self.assertEqual(left.side, "left")

        right_frame = self.frame()
        right_frame[60:160, 185:240] = 255
        right = detector.detect(right_frame, {"found": False, "bbox": None})
        self.assertEqual(right.side, "right")

    def test_draw_debug_returns_same_shape(self) -> None:
        detector = self.build_detector()
        frame = self.frame()
        frame[60:160, 110:210] = 255
        result = detector.detect(frame, {"found": False, "bbox": None})
        debug = detector.draw_debug(frame, result)
        self.assertEqual(debug.shape, frame.shape)


if __name__ == "__main__":
    unittest.main()
