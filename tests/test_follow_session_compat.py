"""Compatibility tests for the FollowSession facade over KernelSession.

Verifies that FollowSession still behaves identically after Step 5:
- run() delegates the lifecycle to KernelSession (takeoff → phases → cleanup),
- send_command routes through the kernel's single RC emission seam (_emit),
- a feature exception triggers the kernel fail-safe instead of escaping.
"""

import unittest
from unittest.mock import patch

import numpy as np

from control.follow_control import FollowController, RCCommand
from control.follow_session import FollowSession, FollowSessionResult
from control.kernel.session import KernelSession
from control.obstacle_avoidance import AvoidanceDecision
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


class LostTargetDetector:
    def detect(self, frame):
        return {
            "found": False,
            "is_predicted": False,
            "ambiguous": False,
            "center": None,
            "area_ratio": 0.0,
            "area": 0,
            "bbox": None,
        }

    def draw_debug(self, frame, result):
        return frame


class ScriptedCamera:
    """Yield numpy frames, then stop the loop after the full sequence."""

    def __init__(self, session, sequence_len) -> None:
        self.session = session
        self.sequence_len = sequence_len
        self.reads = 0

    def read_frame(self):
        self.reads += 1
        if self.reads > self.sequence_len:
            self.session.stop_event.set()
        return np.zeros((480, 640, 3), dtype=np.uint8)


class KernelDrivenSession(FollowSession):
    """Full-lifecycle session with the camera and follow loop faked."""

    def _start_camera(self) -> None:
        self.streaming = True

    def _loop(self) -> None:
        for _ in range(3):
            self.detector.detect(None)


class FollowSessionCompatTestCase(unittest.TestCase):
    def build_session(self, **kwargs):
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        defaults = dict(
            drone=FakeDroneAdapter(verbose_rc=False),
            safety_manager=safety,
            detector=LostTargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )
        defaults.update(kwargs)
        return FollowSession(**defaults)

    def test_run_delegates_full_lifecycle_to_kernel(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        safety = SafetyManager(SafetyConfig(30, 20, 150, 60, 35, 3, 8))
        session = KernelDrivenSession(
            drone=drone,
            safety_manager=safety,
            detector=LostTargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={"display_console_camera": False},
            mode_label="FAKE",
        )
        self.assertIsInstance(session._kernel, KernelSession)
        with patch("control.follow_session.sleep", return_value=None):
            result = session.run()
        self.assertIsInstance(result, FollowSessionResult)
        # 完整生命周期：起飞 → 相位链 → 跟随循环 → 降落清理。
        self.assertEqual(result.state, "FOLLOWING")
        self.assertFalse(result.airborne)
        self.assertEqual(drone.height_cm, 0)  # 已降落

    def test_facade_returns_result_type_unchanged(self) -> None:
        session = self.build_session()
        self.assertIs(session.run().__class__, FollowSessionResult)

    def test_send_command_routes_through_emit_and_zeroes_on_stop(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        session = self.build_session(drone=drone)
        session.stop_event.set()
        session.send_command(RCCommand(50, 30, 20, 10))
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertEqual(session.last_command.as_tuple(), (0, 0, 0, 0))

    def test_emit_zeroes_on_pause(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        session = self.build_session(drone=drone)
        session.paused = True
        session.send_command(RCCommand(50, 30, 20, 10))
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_emit_zeroes_on_emergency_stop(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        session = self.build_session(drone=drone)
        session.emergency_stop = True
        session.send_command(RCCommand(50, 30, 20, 10))
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))

    def test_display_state_reports_only_current_control_owner(self) -> None:
        session = self.build_session()
        session.session_state = "FOLLOWING"
        self.assertEqual(session._display_state(), "FOLLOW")

        session.target_search.state = "LAYER_SCAN_FULL"
        self.assertEqual(session._display_state(), "SEARCH")

        session.last_avoidance_decision = AvoidanceDecision(
            command=RCCommand(),
            state="AVOIDING",
            action="AVOID_RIGHT",
            reason="blocked",
            confidence=1.0,
            plan_id="test",
            observation=ObstacleResult(state="BLOCKED"),
        )
        self.assertEqual(session._display_state(), "OBSTACLE")

        session.paused = True
        self.assertEqual(session._display_state(), "PAUSED")

        session.session_state = "TARGET_LOST_LANDING"
        self.assertEqual(session._display_state(), "LANDING")

    def test_display_state_uses_chinese_labels(self) -> None:
        self.assertEqual(FollowSession._display_state_chinese("FOLLOW"), "跟随")
        self.assertEqual(FollowSession._display_state_chinese("SEARCH"), "搜索")
        self.assertEqual(FollowSession._display_state_chinese("OBSTACLE"), "避障")
        self.assertEqual(FollowSession._display_state_chinese("HOVER"), "悬停")
        self.assertEqual(FollowSession._display_state_chinese("PAUSED"), "暂停")
        self.assertEqual(FollowSession._display_state_chinese("LANDING"), "降落")

    def test_state_label_is_drawn_only_near_top_right(self) -> None:
        session = self.build_session()
        frame = np.zeros((480, 640, 3), dtype=np.uint8)

        rendered = session._draw_state_label(frame, "STATE: 跟随", (0, 255, 0))

        self.assertTrue(np.any(rendered[:80, 360:] != 0))
        self.assertFalse(np.any(rendered[:80, :240] != 0))
        self.assertFalse(np.any(rendered[360:, 400:] != 0))

    def test_feature_exception_triggers_failsafe_then_lands(self) -> None:
        drone = FakeDroneAdapter(verbose_rc=False)
        session = self.build_session(
            drone=drone,
            config={
                "display_console_camera": False,
                "frame_failure_limit": 3,
            },
        )
        session.camera = ScriptedCamera(session, 100)

        def boom(*args, **kwargs):
            raise RuntimeError("feature exploded")

        session._arbitration.arbitrate = boom
        with patch("control.follow_session.sleep", return_value=None):
            session._loop()
        # fail-safe：连续失败达到上限 → 按视频丢失策略降落；异常从未逃逸。
        self.assertEqual(session.session_state, "FRAME_LOST_LANDING")
        self.assertEqual(drone.last_rc_command, (0, 0, 0, 0))
        self.assertIn("feature_error", session.search_reason)


if __name__ == "__main__":
    unittest.main()
