"""Tests for selecting red and ArUco detectors from project configuration."""

import unittest
from unittest.mock import patch

import main
from vision.aruco_detect import ArucoTargetDetector
from vision.detector_factory import create_detector
from vision.target_detect import TargetDetector


class DetectorFactoryTestCase(unittest.TestCase):
    def test_red_config_creates_color_detector(self) -> None:
        detector = create_detector({"vision": {"detector_type": "red"}})
        self.assertIsInstance(detector, TargetDetector)

    def test_aruco_config_creates_aruco_detector(self) -> None:
        detector = create_detector({"vision": {"detector_type": "aruco"}})
        self.assertIsInstance(detector, ArucoTargetDetector)

    def test_missing_vision_config_defaults_to_red(self) -> None:
        self.assertIsInstance(create_detector({}), TargetDetector)

    def test_detector_type_is_normalized(self) -> None:
        detector = create_detector({"vision": {"detector_type": " ArUcO "}})
        self.assertIsInstance(detector, ArucoTargetDetector)

    def test_invalid_detector_type_is_explicit(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported detector_type"):
            create_detector({"vision": {"detector_type": "unknown"}})

    def test_build_system_uses_factory_selected_aruco_detector(self) -> None:
        with patch.object(main, "load_config", return_value={"vision": {"detector_type": "aruco"}}):
            controller = main.build_system(use_fake=True)
        self.assertIsInstance(controller.tools._detector, ArucoTargetDetector)

    def test_fake_adapter_uses_selected_aruco_target(self) -> None:
        drone = main.create_drone_adapter(
            use_fake=True,
            config={
                "vision": {
                    "detector_type": "aruco",
                    "aruco_dictionary": "DICT_4X4_50",
                    "target_marker_id": 23,
                }
            },
        )

        self.assertEqual(drone.detector_type, "aruco")
        self.assertEqual(drone.target_marker_id, 23)


if __name__ == "__main__":
    unittest.main()
