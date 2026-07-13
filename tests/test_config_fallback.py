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


if __name__ == "__main__":
    unittest.main()
