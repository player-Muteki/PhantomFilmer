"""Tests for ArucoTargetDetector.

Run with::

    python3 -m unittest tests.test_aruco_detector -v

Full ArUco marker-detection tests require OpenCV to be installed.
Edge-case and logic tests run regardless.
"""

import sys
import unittest
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

try:
    import cv2
    CV2_AVAILABLE = True
except ModuleNotFoundError:
    CV2_AVAILABLE = False

from vision.aruco_detect import ArucoTargetDetector


# ---- helpers ---------------------------------------------------------------

def _make_aruco_frame(cv2_module, marker_id=23, dict_name=None,
                      img_size=(480, 640), margin=140):
    """Create a grayscale test frame with the given ArUco marker embedded."""
    if dict_name is None:
        dict_name = cv2_module.aruco.DICT_4X4_50
    dictionary = cv2_module.aruco.getPredefinedDictionary(dict_name)
    marker = cv2_module.aruco.generateImageMarker(dictionary, marker_id, 200)
    frame = np.ones(img_size, dtype=np.uint8) * 200
    mh, mw = marker.shape
    y1 = margin
    x1 = (img_size[1] - mw) // 2
    frame[y1:y1 + mh, x1:x1 + mw] = marker
    return frame, (x1, y1, mw, mh)


# ---- tests -----------------------------------------------------------------

class ArucoDetectorInitTestCase(unittest.TestCase):
    """Constructor and configuration tests (no OpenCV required)."""

    def test_default_constructor(self) -> None:
        d = ArucoTargetDetector()
        self.assertEqual(d.dictionary_name, "DICT_4X4_50")
        self.assertEqual(d.target_marker_id, 23)
        self.assertAlmostEqual(d.smoothing_alpha, 0.30)
        self.assertEqual(d.temporary_lost_frames, 3)
        self.assertAlmostEqual(d.min_marker_area, 300.0)

    def test_custom_constructor(self) -> None:
        d = ArucoTargetDetector(
            dictionary_name="DICT_5X5_100",
            target_marker_id=42,
            smoothing_alpha=0.5,
            temporary_lost_frames=5,
            min_marker_area=500.0,
        )
        self.assertEqual(d.dictionary_name, "DICT_5X5_100")
        self.assertEqual(d.target_marker_id, 42)
        self.assertAlmostEqual(d.smoothing_alpha, 0.50)
        self.assertEqual(d.temporary_lost_frames, 5)
        self.assertAlmostEqual(d.min_marker_area, 500.0)

    def test_smoothing_alpha_clamped(self) -> None:
        self.assertAlmostEqual(ArucoTargetDetector(smoothing_alpha=-1).smoothing_alpha, 0.0)
        self.assertAlmostEqual(ArucoTargetDetector(smoothing_alpha=5).smoothing_alpha, 1.0)

    def test_temporary_lost_frames_allows_zero(self) -> None:
        self.assertEqual(ArucoTargetDetector(temporary_lost_frames=0).temporary_lost_frames, 0)
        self.assertEqual(ArucoTargetDetector(temporary_lost_frames=-5).temporary_lost_frames, 0)

    @unittest.skipIf(not CV2_AVAILABLE, "cv2 not available")
    def test_from_config_full_vision_section(self) -> None:
        d = ArucoTargetDetector.from_config({
            "vision": {
                "detector_type": "aruco",
                "aruco_dictionary": "DICT_6X6_250",
                "target_marker_id": 7,
                "smoothing_alpha": 0.4,
                "temporary_lost_frames": 10,
                "min_marker_area": 600.0,
            },
        })
        self.assertEqual(d.dictionary_name, "DICT_6X6_250")
        self.assertEqual(d.target_marker_id, 7)

    def test_from_config_fallback_top_level(self) -> None:
        d = ArucoTargetDetector.from_config({
            "detector_type": "aruco",
            "smoothing_alpha": 0.15,
        })
        self.assertAlmostEqual(d.smoothing_alpha, 0.15)

    def test_from_config_empty_uses_defaults(self) -> None:
        d = ArucoTargetDetector.from_config({})
        self.assertEqual(d.target_marker_id, 23)
        self.assertAlmostEqual(d.smoothing_alpha, 0.30)

    def test_reset_clears_state(self) -> None:
        d = ArucoTargetDetector()
        d._smooth_initialized = True
        d._smooth_x = 100.0
        d._lost_count = 5
        d.reset()
        self.assertFalse(d._smooth_initialized)
        self.assertIsNone(d._smooth_x)
        self.assertEqual(d._lost_count, 0)


class ArucoDetectorEdgeCaseTestCase(unittest.TestCase):
    """Tests that do not require OpenCV (frame validation, empty results)."""

    def setUp(self):
        self.detector = ArucoTargetDetector()

    def test_detect_none_returns_empty(self) -> None:
        r = self.detector.detect(None)
        self.assertFalse(r["found"])
        self.assertEqual(r["area"], 0.0)
        self.assertIsNone(r["center"])
        self.assertIsNone(r["bbox"])
        self.assertIsNone(r["marker_id"])
        self.assertEqual(r["detector_type"], "aruco")

    def test_detect_empty_array_returns_empty(self) -> None:
        r = self.detector.detect(np.array([], dtype=np.uint8))
        self.assertFalse(r["found"])

    def test_detect_string_returns_empty(self) -> None:
        r = self.detector.detect("not a frame")
        self.assertFalse(r["found"])

    def test_detect_list_returns_empty(self) -> None:
        r = self.detector.detect([1, 2, 3])
        self.assertFalse(r["found"])

    def test_detect_1d_array_returns_empty(self) -> None:
        r = self.detector.detect(np.array([1, 2, 3], dtype=np.uint8))
        self.assertFalse(r["found"])

    def test_detect_zero_size_array_returns_empty(self) -> None:
        r = self.detector.detect(np.zeros((0, 10), dtype=np.uint8))
        self.assertFalse(r["found"])

    def test_empty_result_has_required_fields(self) -> None:
        r = self.detector.detect(None)
        for key in ("found", "center", "target_center_x", "target_center_y",
                    "area", "bbox", "marker_id", "corners", "detector_type"):
            self.assertIn(key, r)

    def test_draw_debug_none_returns_none(self) -> None:
        result = self.detector.detect(None)
        self.assertIsNone(self.detector.draw_debug(None, result))

    def test_draw_debug_empty_array(self) -> None:
        result = self.detector.detect(None)
        out = self.detector.draw_debug(np.array([], dtype=np.uint8), result)
        self.assertIsNotNone(out)
        self.assertEqual(out.size, 0)

    @unittest.skipIf(not CV2_AVAILABLE, "cv2 not available — skipping frame-lost debug test")
    def test_draw_debug_valid_frame_lost(self) -> None:
        result = self.detector.detect(np.ones((480, 640, 3), dtype=np.uint8) * 128)
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 200
        out = self.detector.draw_debug(frame, result)
        self.assertEqual(out.shape, frame.shape)

    def test_reset_no_cv2_required(self) -> None:
        d = ArucoTargetDetector()
        d.reset()  # must not raise


@unittest.skipIf(not CV2_AVAILABLE, "cv2 not available — skipping ArUco detection tests")
class ArucoDetectorDetectionTestCase(unittest.TestCase):
    """Full detection tests — each test gets a fresh detector and frame."""

    @classmethod
    def setUpClass(cls):
        cls.DICT = cv2.aruco.DICT_4X4_50
        cls.marker_23 = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cls.DICT), 23, 200
        )

    def setUp(self):
        super().setUp()
        self.detector = ArucoTargetDetector()
        self.IMG_SIZE = (480, 640)
        frame = np.ones(self.IMG_SIZE, dtype=np.uint8) * 200
        mh, mw = self.marker_23.shape
        x1 = (self.IMG_SIZE[1] - mw) // 2
        frame[140:140 + mh, x1:x1 + mw] = self.marker_23
        self.frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        self.blank = np.ones(self.IMG_SIZE + (3,), dtype=np.uint8) * 128

    def test_detects_target_marker(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        self.assertTrue(r["found"])
        self.assertIsNotNone(r["center"])
        self.assertIsNotNone(r["bbox"])
        self.assertGreater(r["area"], 0)
        self.assertEqual(r["marker_id"], 23)
        self.assertEqual(r["detector_type"], "aruco")
        self.assertFalse(r["is_predicted"])

    def test_center_within_bbox(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        cx, cy = r["center"]
        x, y, w, h = r["bbox"]
        self.assertGreaterEqual(cx, x)
        self.assertGreaterEqual(cy, y)
        self.assertLessEqual(cx, x + w)
        self.assertLessEqual(cy, y + h)

    def test_area_is_float_and_positive(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        self.assertIsInstance(r["area"], float)
        self.assertGreater(r["area"], 0)

    def test_bbox_reasonable(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        x, y, w, h = r["bbox"]
        self.assertGreater(w, 0)
        self.assertGreater(h, 0)
        frame_area = 480 * 640
        pixel_ratio = (w * h) / frame_area
        self.assertGreater(pixel_ratio, 0.01)
        self.assertLess(pixel_ratio, 0.90)

    def test_no_marker_returns_not_found(self) -> None:
        """Blank frame with no prior detection must return found=False."""
        det = ArucoTargetDetector()
        r = det.detect(self.blank)
        self.assertFalse(r["found"])
        self.assertEqual(r["area"], 0.0)

    def test_different_marker_id_ignored(self) -> None:
        """A marker with a non-target ID should not be detected."""
        det = ArucoTargetDetector()
        frame, _ = _make_aruco_frame(cv2, marker_id=7)
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        r = det.detect(frame)
        self.assertFalse(r["found"])

    def test_min_marker_area_filters(self) -> None:
        small = ArucoTargetDetector(min_marker_area=1e9)
        r = small.detect(self.frame_bgr)
        self.assertFalse(r["found"])

    def test_multiple_markers_picks_correct_id(self) -> None:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        m23 = cv2.aruco.generateImageMarker(dictionary, 23, 100)
        m7 = cv2.aruco.generateImageMarker(dictionary, 7, 100)
        frame = np.ones((480, 640, 3), dtype=np.uint8) * 200
        frame[50:150, 50:150] = cv2.cvtColor(m23, cv2.COLOR_GRAY2BGR)
        frame[50:150, 300:400] = cv2.cvtColor(m7, cv2.COLOR_GRAY2BGR)
        r = self.detector.detect(frame)
        self.assertTrue(r["found"])
        self.assertEqual(r["marker_id"], 23)

    def test_corners_provided_when_found(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        self.assertIsNotNone(r["corners"])
        self.assertEqual(len(r["corners"]), 4)

    def test_draw_debug_found_returns_frame(self) -> None:
        r = self.detector.detect(self.frame_bgr)
        out = self.detector.draw_debug(self.frame_bgr, r)
        self.assertEqual(out.shape, self.frame_bgr.shape)

    def test_draw_debug_lost_returns_frame(self) -> None:
        r = self.detector.detect(self.blank)
        out = self.detector.draw_debug(self.blank, r)
        self.assertEqual(out.shape, self.blank.shape)


@unittest.skipIf(not CV2_AVAILABLE, "cv2 not available — skipping smoothing tests")
class ArucoDetectorSmoothingTestCase(unittest.TestCase):
    """Verify exponential moving-average smoothing behaviour."""

    @classmethod
    def setUpClass(cls):
        cls.marker_img = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 23, 200
        )

    def setUp(self):
        super().setUp()
        frame = np.ones((480, 640), dtype=np.uint8) * 200
        frame[140:340, 220:420] = self.marker_img
        self.frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    def test_first_detection_initialises_smooth(self) -> None:
        detector = ArucoTargetDetector(smoothing_alpha=0.50)
        r1 = detector.detect(self.frame_bgr)
        self.assertTrue(r1["found"])
        self.assertIsNotNone(r1["center"])

    def test_smoothed_values_converge(self) -> None:
        detector = ArucoTargetDetector(smoothing_alpha=0.30)
        _ = detector.detect(self.frame_bgr)
        r2 = detector.detect(self.frame_bgr)
        r3 = detector.detect(self.frame_bgr)
        self.assertLessEqual(abs(r2["center"][0] - r3["center"][0]), 1)
        self.assertLessEqual(abs(r2["center"][1] - r3["center"][1]), 1)
        if r2["area"] > 0 and r3["area"] > 0:
            self.assertLessEqual(abs(r2["area"] - r3["area"]) / max(r2["area"], r3["area"]), 0.01)


@unittest.skipIf(not CV2_AVAILABLE, "cv2 not available — skipping temp-lost tests")
class ArucoDetectorTempLostTestCase(unittest.TestCase):
    """Verify temporary-loss handling."""

    @classmethod
    def setUpClass(cls):
        cls.marker_img = cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50), 23, 200
        )

    def _detect_frame(self):
        frame = np.ones((480, 640), dtype=np.uint8) * 200
        frame[140:340, 220:420] = self.marker_img
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

    def _blank_frame(self):
        return np.ones((480, 640, 3), dtype=np.uint8) * 128

    def test_initial_lost_returns_stale_values(self) -> None:
        detector = ArucoTargetDetector(temporary_lost_frames=3)
        r1 = detector.detect(self._detect_frame())
        self.assertTrue(r1["found"])

        r2 = detector.detect(self._blank_frame())
        self.assertTrue(r2["found"], "Should still report found within temp-lost window")
        self.assertTrue(r2["is_predicted"])
        self.assertEqual(r2["center"], r1["center"])

    def test_exceeds_threshold_reports_lost(self) -> None:
        detector = ArucoTargetDetector(temporary_lost_frames=2)
        _ = detector.detect(self._detect_frame())
        blank = self._blank_frame()
        _ = detector.detect(blank)
        _ = detector.detect(blank)
        r3 = detector.detect(blank)
        self.assertFalse(r3["found"])
        self.assertFalse(r3["is_predicted"])

    def test_zero_temporary_lost_frames_reports_lost_immediately(self) -> None:
        detector = ArucoTargetDetector(temporary_lost_frames=0)
        detected = detector.detect(self._detect_frame())
        self.assertTrue(detected["found"])
        self.assertFalse(detected["is_predicted"])

        lost = detector.detect(self._blank_frame())
        self.assertFalse(lost["found"])
        self.assertFalse(lost["is_predicted"])

    def test_recovery_after_temp_lost(self) -> None:
        detector = ArucoTargetDetector(temporary_lost_frames=3)
        _ = detector.detect(self._detect_frame())
        _ = detector.detect(self._blank_frame())
        r_recover = detector.detect(self._detect_frame())
        self.assertTrue(r_recover["found"])
        self.assertEqual(r_recover["marker_id"], 23)


if __name__ == "__main__":
    unittest.main()
