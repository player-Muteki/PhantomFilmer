"""Tests for the dependency-free config fallback parser."""

import tempfile
import unittest
from pathlib import Path

import main


class ConfigFallbackTestCase(unittest.TestCase):
    def test_vision_block_is_available_without_pyyaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                "vision:\n  detector_type: aruco\n  temporary_lost_frames: 0\n",
                encoding="utf-8",
            )
            config = main._load_config_without_yaml(path)
        self.assertEqual(config["vision"]["detector_type"], "aruco")
        self.assertEqual(config["vision"]["temporary_lost_frames"], 0)
    def test_obstacle_block_is_available_without_pyyaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                "obstacle:\n  enabled: true\n  recovery_clear_frames: 4\n  forward_speed_in_caution_ratio: 0.25\n",
                encoding="utf-8",
            )
            config = main._load_config_without_yaml(path)
        self.assertTrue(config["obstacle"]["enabled"])
        self.assertEqual(config["obstacle"]["recovery_clear_frames"], 4)
        self.assertEqual(config["obstacle"]["forward_speed_in_caution_ratio"], 0.25)

    def test_target_search_block_is_available_without_pyyaml(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.yaml"
            path.write_text(
                "target_search:\n  enabled: true\n  yaw_speed: 20\n",
                encoding="utf-8",
            )
            config = main._load_config_without_yaml(path)
        self.assertTrue(config["target_search"]["enabled"])
        self.assertEqual(config["target_search"]["yaw_speed"], 20)


if __name__ == "__main__":
    unittest.main()
