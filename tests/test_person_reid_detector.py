"""Dependency-free tests for the person ReID detector policy and contract."""

import unittest

import numpy as np

from vision.person_reid_detect import PersonReIDDetector


class FakePersonDetector:
    def __init__(self, detections):
        self.detections = detections

    def detect_people(self, frame):
        return list(self.detections)


class FakeFeatureExtractor:
    def __init__(self, features):
        self.features = np.asarray(features, dtype=np.float32)

    def extract(self, images):
        return self.features[: len(images)]


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
        self.assertEqual(detector.similarity_threshold, 0.72)
        self.assertEqual(detector.ambiguity_margin, 0.08)


if __name__ == "__main__":
    unittest.main()
