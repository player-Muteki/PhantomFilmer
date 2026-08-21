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


class FakeOrientationEstimator:
    def __init__(self):
        self.prepare_calls = 0
        self.reset_calls = 0
        self.estimate_calls = []

    def prepare(self):
        self.prepare_calls += 1

    def estimate(self, frame, bbox):
        self.estimate_calls.append((frame, bbox))
        return {
            "body_orientation_model": "jointbdoe",
            "body_orientation_angle": 92.5,
            "body_orientation_raw_angle": 95.0,
            "body_orientation_detection_confidence": 0.88,
            "body_orientation_match_iou": 0.74,
            "body_orientation_latency_ms": 12.0,
        }

    def reset(self):
        self.reset_calls += 1


class FakeYOLO:
    def __init__(self, model_path):
        self.model_path = model_path


class FakeSceneYOLO:
    names = {0: "person", 56: "chair", 2: "car"}

    def __init__(self, model_path):
        self.model_path = model_path
        self.calls = []

    def predict(self, **kwargs):
        self.calls.append(kwargs)
        values = SimpleNamespace(
            xyxy=SimpleNamespace(
                detach=lambda: SimpleNamespace(
                    cpu=lambda: SimpleNamespace(
                        numpy=lambda: np.array(
                            [[5, 10, 35, 90], [40, 20, 90, 80], [10, 30, 50, 70]],
                            dtype=np.float32,
                        )
                    )
                )
            ),
            conf=SimpleNamespace(
                detach=lambda: SimpleNamespace(
                    cpu=lambda: SimpleNamespace(
                        numpy=lambda: np.array([0.9, 0.8, 0.2], dtype=np.float32)
                    )
                )
            ),
            cls=SimpleNamespace(
                detach=lambda: SimpleNamespace(
                    cpu=lambda: SimpleNamespace(
                        numpy=lambda: np.array([0, 56, 2], dtype=np.float32)
                    )
                )
            ),
        )
        return [SimpleNamespace(boxes=values)]


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

    def test_yolo_scene_detection_separates_people_and_visual_objects(self):
        fake_ultralytics = SimpleNamespace(YOLO=FakeSceneYOLO)
        with patch.dict(sys.modules, {"ultralytics": fake_ultralytics}):
            detector = UltralyticsPersonDetector(
                "local.pt",
                0.45,
                "cpu",
                visual_object_detection_enabled=True,
                visual_object_confidence=0.35,
                visual_object_classes=("chair", "car"),
            )
            scene = detector.detect_scene(self.frame)

        self.assertEqual(len(scene["people"]), 1)
        self.assertEqual(len(scene["visual_objects"]), 1)
        self.assertEqual(scene["visual_objects"][0]["display_label"], "障碍物候选：椅子")
        self.assertIsNone(detector.model.calls[0]["classes"])

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
        self.assertEqual(result["candidates"][0]["display_label"], "非目标人物")
        self.assertEqual(result["candidates"][1]["display_label"], "目标人物")

    def test_jointbdoe_runs_only_after_reid_accepts_target(self):
        orientation = FakeOrientationEstimator()
        detector = build_detector(
            [
                {"bbox_xyxy": (5, 10, 35, 90)},
                {"bbox_xyxy": (60, 20, 110, 90)},
            ],
            [[0.0, 1.0], [1.0, 0.0]],
            orientation_estimator=orientation,
        )

        result = detector.detect(self.frame)

        self.assertTrue(result["found"])
        self.assertEqual(result["body_orientation_model"], "jointbdoe")
        self.assertEqual(result["body_orientation_angle"], 92.5)
        self.assertEqual(result["body_orientation_detection_confidence"], 0.88)
        self.assertEqual(orientation.prepare_calls, 1)
        self.assertEqual(orientation.estimate_calls[0][1], (60, 20, 50, 70))

        detector.reset()
        self.assertEqual(orientation.reset_calls, 1)

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
        annotations = []
        labels = []
        fake_cv2 = SimpleNamespace(
            FONT_HERSHEY_SIMPLEX=0,
            line=lambda *args, **kwargs: None,
            putText=lambda image, label, *args, **kwargs: labels.append(label),
        )

        def capture_annotations(frame, values):
            annotations.extend(list(values))
            return frame

        with patch.dict(sys.modules, {"cv2": fake_cv2}), patch(
            "vision.person_reid_detect.draw_box_annotations",
            side_effect=capture_annotations,
        ):
            detector.draw_debug(self.frame, result)

        self.assertEqual(len(annotations), 2)
        self.assertTrue(any(item.label == "非目标人物 0.800" for item in annotations))
        self.assertTrue(any(item.label == "非目标人物 0.600" for item in annotations))

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
