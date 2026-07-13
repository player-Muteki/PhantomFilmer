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

    def test_fake_aruco_follow_exits_before_connecting(self) -> None:
        config = {"vision": {"detector_type": "aruco"}}
        with patch.object(main, "load_config", return_value=config), patch.object(
            main, "create_drone_adapter"
        ) as create_adapter:
            self.assertEqual(main.run_follow(use_fake=True), 0)
        create_adapter.return_value.connect.assert_not_called()


if __name__ == "__main__":
    unittest.main()
