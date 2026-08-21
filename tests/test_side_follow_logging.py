"""Tests for loss-tolerant side-follow JSONL flight telemetry."""

import json
import tempfile
import unittest
from pathlib import Path

from control.follow_control import RCCommand
from control.side_follow_control import SideFollowDebugInfo
from control.side_follow_logging import SideFollowEventRecorder, SideFollowLogConfig


class SideFollowEventRecorderTestCase(unittest.TestCase):
    def test_writes_control_decision_and_flight_telemetry(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SideFollowEventRecorder(
                SideFollowLogConfig(enabled=True, log_dir=Path(tmp))
            )
            recorder.reset("REID-DEMO")
            recorder.record(
                mode="REID-DEMO",
                target_result={
                    "found": True,
                    "center": (500, 240),
                    "bbox": (450, 180, 100, 120),
                    "area": 12_000.0,
                    "similarity": 0.91,
                    "body_orientation_angle": 72.5,
                    "body_orientation_detection_confidence": 0.88,
                    "body_orientation_match_iou": 0.76,
                },
                debug=SideFollowDebugInfo(
                    state="SIDE_POSITION_TRACKING",
                    current_angle=72.5,
                    selected_angle=90,
                    angle_error=17.5,
                    stable_samples=5,
                    lock_frames=1,
                    orbit_direction="COUNTERCLOCKWISE",
                    yaw_feedforward=-12,
                    yaw_feedback=4,
                    side_locked=True,
                    position_priority=True,
                    side_reselect_pending=True,
                    center_tolerance_ratio=0.056,
                    horizontal_error=0.5625,
                    tracking_lateral=20,
                    orbit_lateral=0,
                ),
                command=RCCommand(20, 16, 0, 0),
                state="SIDE_POSITION_TRACKING",
                reason="position priority",
                frame_width=640,
                frame_height=480,
                battery_percent=67,
                height_cm=149,
                aircraft_yaw_deg=-32,
                control_hz=18.4,
                vision_fps=17.9,
            )
            recorder.close()

            files = list(Path(tmp).glob("*.jsonl"))
            self.assertEqual(len(files), 1)
            payload = json.loads(files[0].read_text(encoding="utf-8").strip())
            self.assertEqual(payload["event"], "side_follow_decision")
            self.assertTrue(payload["session_id"])
            self.assertEqual(payload["target"]["center_norm"], [0.7812, 0.5])
            self.assertEqual(payload["side_follow"]["selected_angle"], 90)
            self.assertTrue(payload["side_follow"]["position_priority"])
            self.assertTrue(payload["side_follow"]["side_reselect_pending"])
            self.assertEqual(payload["side_follow"]["center_tolerance_ratio"], 0.056)
            self.assertEqual(payload["side_follow"]["tracking_lateral"], 20)
            self.assertEqual(payload["recovery"]["state"], "IDLE")
            self.assertEqual(payload["recovery"]["horizontal_direction"], 0)
            self.assertEqual(payload["flight"]["battery_percent"], 67)
            self.assertEqual(payload["flight"]["height_cm"], 149)
            self.assertEqual(payload["flight"]["aircraft_yaw_deg"], -32)
            self.assertEqual(
                payload["final_command"],
                {"left_right": 20, "forward_backward": 16, "up_down": 0, "yaw": 0},
            )

    def test_is_disabled_unless_explicitly_enabled(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            recorder = SideFollowEventRecorder(
                SideFollowLogConfig.from_config({"side_follow": {"log_dir": tmp}})
            )
            recorder.reset("TEST")

            self.assertIsNone(recorder.log_path)

    def test_sampling_and_queue_settings_are_bounded(self) -> None:
        config = SideFollowLogConfig.from_config(
            {
                "side_follow": {
                    "log_enabled": True,
                    "log_every_n_frames": 0,
                    "log_queue_size": -2,
                }
            }
        )

        self.assertTrue(config.enabled)
        self.assertEqual(config.log_every_n_frames, 1)
        self.assertEqual(config.queue_size, 1)


if __name__ == "__main__":
    unittest.main()
