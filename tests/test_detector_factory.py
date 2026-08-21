"""Tests for the single person ReID detector factory."""

import unittest

from vision.detector_factory import create_detector
from vision.person_reid_detect import PersonReIDDetector


class DetectorFactoryTestCase(unittest.TestCase):
    def test_factory_creates_reid_detector_without_loading_models(self) -> None:
        detector = create_detector({"vision": {}})

        self.assertIsInstance(detector, PersonReIDDetector)

    def test_missing_vision_config_still_creates_reid_detector(self) -> None:
        self.assertIsInstance(create_detector({}), PersonReIDDetector)

if __name__ == "__main__":
    unittest.main()
