"""Dependency-light tests for JointBDOE matching and angle smoothing."""

import unittest

import numpy as np

from vision.jointbdoe_orientation import (
    JointBDOEOrientationEstimator,
    JointBDOEPrediction,
    _bbox_iou,
    _circular_mean_deg,
)


class JointBDOEOrientationTestCase(unittest.TestCase):
    def test_circular_mean_handles_wraparound(self):
        angle = _circular_mean_deg([359.0, 1.0])
        self.assertTrue(angle < 1.0 or angle > 359.0)

    def test_bbox_iou(self):
        self.assertAlmostEqual(_bbox_iou((0, 0, 10, 10), (5, 0, 15, 10)), 1 / 3)
        self.assertEqual(_bbox_iou((0, 0, 2, 2), (3, 3, 4, 4)), 0.0)

    def test_estimate_matches_reid_box_and_exposes_honest_confidence_fields(self):
        estimator = JointBDOEOrientationEstimator(
            match_iou_threshold=0.2,
            smoothing_window=2,
        )
        frame = np.zeros((100, 120, 3), dtype=np.uint8)
        predictions = iter(
            [
                [
                    JointBDOEPrediction((60, 20, 110, 90), 359.0, 0.88),
                    JointBDOEPrediction((0, 0, 20, 20), 180.0, 0.99),
                ],
                [JointBDOEPrediction((60, 20, 110, 90), 1.0, 0.90)],
            ]
        )
        estimator.detect_people = lambda _frame: next(predictions)

        first = estimator.estimate(frame, (60, 20, 50, 70))
        second = estimator.estimate(frame, (60, 20, 50, 70))

        self.assertEqual(first["body_orientation_model"], "jointbdoe")
        self.assertAlmostEqual(first["body_orientation_raw_angle"], 359.0)
        self.assertAlmostEqual(first["body_orientation_detection_confidence"], 0.88)
        self.assertAlmostEqual(first["body_orientation_match_iou"], 1.0)
        self.assertTrue(
            second["body_orientation_angle"] < 1.0
            or second["body_orientation_angle"] > 359.0
        )
        self.assertNotIn("body_orientation_confidence", second)

    def test_estimate_rejects_weak_box_match(self):
        estimator = JointBDOEOrientationEstimator(match_iou_threshold=0.5)
        estimator.detect_people = lambda _frame: [
            JointBDOEPrediction((0, 0, 10, 10), 30.0, 0.9)
        ]
        frame = np.zeros((100, 120, 3), dtype=np.uint8)
        self.assertIsNone(estimator.estimate(frame, (60, 20, 50, 70)))


if __name__ == "__main__":
    unittest.main()
