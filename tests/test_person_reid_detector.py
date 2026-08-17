"""Dependency-free tests for the person ReID detector policy and contract."""

import os
import sys
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from vision.person_reid_detect import PersonReIDDetector, UltralyticsPersonDetector


class FakePersonDetector:
    def __init__(self, detections):
        self.detections = detections

    def detect_people(self, frame):
        return list(self.detections)


class FakeFeatureExtractor:
    def __init__(self, features):
        self.features = np.asarray(features, dtype=np.float32)
        self.last_images = []

    def extract(self, images):
        self.last_images = list(images)
        return self.features[: len(images)]


class FakeYOLO:
    def __init__(self, model_path):
        self.model_path = model_path


def build_detector(detections, features, **kwargs):
    return PersonReIDDetector(
        reference_image_paths=[],
        person_detector=FakePersonDetector(detections),
        feature_extractor=FakeFeatureExtractor(features),
        reference_features=np.array([[1.0, 0.0]], dtype=np.float32),
        similarity_threshold=kwargs.pop("similarity_threshold", 0.65),
        ambiguity_margin=kwargs.pop("ambiguity_margin", 0.05),
        **kwargs,
    )


class PersonReIDDetectorTestCase(unittest.TestCase):
    def setUp(self):
        self.frame = np.zeros((100, 120, 3), dtype=np.uint8)

    def test_yolo_defaults_to_offline_mode(self):
        fake_ultralytics = SimpleNamespace(YOLO=FakeYOLO)
        with patch.dict(os.environ, {}, clear=False):
            os.environ.pop("YOLO_OFFLINE", None)
            with patch.dict(sys.modules, {"ultralytics": fake_ultralytics}):
                detector = UltralyticsPersonDetector("local.pt", 0.5, "cpu")
            self.assertEqual(os.environ["YOLO_OFFLINE"], "1")
            self.assertEqual(detector.model.model_path, "local.pt")

    def test_yolo_offline_default_respects_explicit_override(self):
        fake_ultralytics = SimpleNamespace(YOLO=FakeYOLO)
        with patch.dict(os.environ, {"YOLO_OFFLINE": "0"}, clear=False):
            with patch.dict(sys.modules, {"ultralytics": fake_ultralytics}):
                UltralyticsPersonDetector("local.pt", 0.5, "cpu")
            self.assertEqual(os.environ["YOLO_OFFLINE"], "0")

    def test_selects_person_with_best_reference_similarity(self):
        detector = build_detector(
            [
                {"bbox_xyxy": (5, 10, 35, 90)},
                {"bbox_xyxy": (60, 20, 110, 90)},
            ],
            [[0.0, 1.0], [0.95, 0.05]],
        )
        result = detector.detect(self.frame)
        self.assertTrue(result["found"])
        self.assertEqual(result["bbox"], (60, 20, 50, 70))
        self.assertEqual(result["center"], (85, 55))
        self.assertGreater(result["similarity"], 0.99)
        self.assertEqual(result["candidate_count"], 2)

    def test_below_threshold_is_rejected(self):
        detector = build_detector(
            [{"bbox_xyxy": (5, 10, 35, 90)}],
            [[0.0, 1.0]],
            similarity_threshold=0.7,
        )
        result = detector.detect(self.frame)
        self.assertFalse(result["found"])
        self.assertAlmostEqual(result["similarity"], 0.0)
        self.assertEqual(result["candidate_count"], 1)
        self.assertEqual(len(result["candidates"]), 1)
        self.assertEqual(result["candidates"][0]["bbox"], (5, 10, 30, 80))
        self.assertAlmostEqual(result["candidates"][0]["similarity"], 0.0)
        self.assertEqual(result["similarity_threshold"], 0.7)

    def test_ambiguous_top_two_people_are_rejected(self):
        detector = build_detector(
            [
                {"bbox_xyxy": (5, 10, 35, 90)},
                {"bbox_xyxy": (60, 20, 110, 90)},
            ],
            [[1.0, 0.01], [1.0, 0.02]],
            ambiguity_margin=0.05,
        )
        result = detector.detect(self.frame)
        self.assertFalse(result["found"])
        self.assertTrue(result["ambiguous"])
        self.assertEqual(result["candidate_count"], 2)
        self.assertEqual(len(result["candidates"]), 2)

    def test_debug_overlay_draws_every_yolo_candidate_and_scores(self):
        detector = build_detector(
            [
                {"bbox_xyxy": (5, 10, 35, 90)},
                {"bbox_xyxy": (60, 20, 110, 90)},
            ],
            [[0.6, 0.8], [0.8, 0.6]],
            similarity_threshold=0.9,
        )
        result = detector.detect(self.frame)
        rectangles = []
        labels = []
        fake_cv2 = SimpleNamespace(
            FONT_HERSHEY_SIMPLEX=0,
            line=lambda *args, **kwargs: None,
            rectangle=lambda *args, **kwargs: rectangles.append(args),
            putText=lambda image, label, *args, **kwargs: labels.append(label),
        )

        with patch.dict(sys.modules, {"cv2": fake_cv2}):
            detector.draw_debug(self.frame, result)

        self.assertEqual(len(rectangles), 2)
        self.assertTrue(any("#1 ReID=" in label and "AREA=" in label for label in labels))
        self.assertTrue(any("#2 ReID=" in label and "AREA=" in label for label in labels))
        self.assertTrue(
            any("YOLO people=2" in label and "threshold=0.900" in label for label in labels)
        )
        self.assertTrue(any(label.startswith("ReID BELOW THRESHOLD") for label in labels))

    def test_candidate_area_ratio_uses_follow_distance_thresholds(self):
        detector = build_detector(
            [
                {"bbox_xyxy": (0, 0, 10, 30)},
                {"bbox_xyxy": (20, 0, 40, 30)},
                {"bbox_xyxy": (50, 0, 80, 40)},
            ],
            [[1.0, 0.0], [0.8, 0.6], [0.6, 0.8]],
            target_area_ratio_min=0.03,
            target_area_ratio_max=0.08,
        )

        result = detector.detect(self.frame)

        self.assertEqual(
            [candidate["distance_state"] for candidate in result["candidates"]],
            ["FAR", "OK", "NEAR"],
        )
        self.assertAlmostEqual(result["candidates"][0]["area_ratio"], 0.025)
        self.assertAlmostEqual(result["candidates"][1]["area_ratio"], 0.05)
        self.assertAlmostEqual(result["candidates"][2]["area_ratio"], 0.1)

    def test_temporary_loss_is_marked_predicted(self):
        person_detector = FakePersonDetector([{"bbox_xyxy": (5, 10, 35, 90)}])
        detector = PersonReIDDetector(
            reference_image_paths=[],
            person_detector=person_detector,
            feature_extractor=FakeFeatureExtractor([[1.0, 0.0]]),
            reference_features=[[1.0, 0.0]],
            temporary_lost_frames=1,
        )
        first = detector.detect(self.frame)
        person_detector.detections = []
        predicted = detector.detect(self.frame)
        lost = detector.detect(self.frame)
        self.assertTrue(first["found"])
        self.assertTrue(predicted["found"])
        self.assertTrue(predicted["is_predicted"])
        self.assertFalse(lost["found"])

    def test_reset_removes_stale_prediction(self):
        person_detector = FakePersonDetector([{"bbox_xyxy": (5, 10, 35, 90)}])
        detector = PersonReIDDetector(
            reference_image_paths=[],
            person_detector=person_detector,
            feature_extractor=FakeFeatureExtractor([[1.0, 0.0]]),
            reference_features=[[1.0, 0.0]],
            temporary_lost_frames=3,
        )
        self.assertTrue(detector.detect(self.frame)["found"])
        detector.reset()
        person_detector.detections = []
        self.assertFalse(detector.detect(self.frame)["found"])

    def test_invalid_frame_returns_safe_empty_result(self):
        detector = PersonReIDDetector(reference_image_paths=[])
        result = detector.detect(None)
        self.assertFalse(result["found"])
        self.assertEqual(result["detector_type"], "person_reid")

    def test_from_config_reads_comma_separated_reference_images(self):
        detector = PersonReIDDetector.from_config(
            {
                "vision": {
                    "reference_images": "front.jpg, side.jpg",
                    "reid_similarity_threshold": 0.72,
                    "reid_ambiguity_margin": 0.08,
                }
            }
        )
        self.assertEqual(detector.reference_image_paths, ["front.jpg", "side.jpg"])
        self.assertEqual(detector.detector_model_path, "weights/yolo11n.pt")
        self.assertEqual(detector.similarity_threshold, 0.72)
        self.assertEqual(detector.ambiguity_margin, 0.08)

    def test_from_config_loads_persistent_reference_profile(self):
        with patch(
            "vision.reid_profiles.load_reid_profile",
            return_value=(np.array([1.0, 0.0], dtype=np.float32), {"profile_name": "person-a"}),
        ) as load_profile:
            detector = PersonReIDDetector.from_config(
                {"vision": {"reference_profile": "person-a"}}
            )

        load_profile.assert_called_once()
        self.assertEqual(detector.reference_image_paths, [])
        self.assertTrue(np.allclose(detector.reference_feature, [1.0, 0.0]))

    def test_reference_photo_is_cropped_to_detected_person(self) -> None:
        with TemporaryDirectory() as temp_dir:
            photo = Path(temp_dir) / "person.jpg"
            photo.write_bytes(b"fake image")
            extractor = FakeFeatureExtractor([[1.0, 0.0]])
            detector = PersonReIDDetector(
                reference_image_paths=[str(photo)],
                person_detector=FakePersonDetector(
                    [{"bbox_xyxy": (30, 10, 110, 90)}]
                ),
                feature_extractor=extractor,
            )

            fake_cv2 = SimpleNamespace(
                imread=lambda path: np.zeros((100, 120, 3), dtype=np.uint8)
            )
            with patch.dict("sys.modules", {"cv2": fake_cv2}):
                detector.prepare()
            self.assertEqual(extractor.last_images[0].shape, (80, 80, 3))

    def test_reference_photo_with_multiple_people_is_rejected(self) -> None:
        with TemporaryDirectory() as temp_dir:
            photo = Path(temp_dir) / "crowd.jpg"
            photo.write_bytes(b"fake image")
            detector = PersonReIDDetector(
                reference_image_paths=[str(photo)],
                person_detector=FakePersonDetector(
                    [
                        {"bbox_xyxy": (0, 0, 20, 20)},
                        {"bbox_xyxy": (30, 10, 110, 90)},
                    ]
                ),
                feature_extractor=FakeFeatureExtractor([[1.0, 0.0]]),
            )
            fake_cv2 = SimpleNamespace(
                imread=lambda path: np.zeros((100, 120, 3), dtype=np.uint8)
            )
            with patch.dict("sys.modules", {"cv2": fake_cv2}), self.assertRaisesRegex(
                RuntimeError, "检测到多个人"
            ):
                detector.prepare()

    def test_reference_photo_without_person_is_rejected(self) -> None:
        with TemporaryDirectory() as temp_dir:
            photo = Path(temp_dir) / "empty.jpg"
            photo.write_bytes(b"fake image")
            detector = PersonReIDDetector(
                reference_image_paths=[str(photo)],
                person_detector=FakePersonDetector([]),
                feature_extractor=FakeFeatureExtractor([[1.0, 0.0]]),
            )
            fake_cv2 = SimpleNamespace(
                imread=lambda path: np.zeros((100, 120, 3), dtype=np.uint8)
            )
            with patch.dict("sys.modules", {"cv2": fake_cv2}), self.assertRaisesRegex(
                RuntimeError, "未检测到完整人物"
            ):
                detector.prepare()


if __name__ == "__main__":
    unittest.main()
