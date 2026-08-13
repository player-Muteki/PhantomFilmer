"""Integration tests for occlusion-aware behavior during target loss.

These drive FollowSession._loop directly and verify the final RC channels: an
unlocked mission may only yaw, while a locked target plus a persistent overlapping
blocker may trigger a lateral-only peek.
"""

import unittest
from unittest.mock import patch

import numpy as np

from control.follow_control import FollowController
from control.follow_session import FollowSession
from control.motion_arbiter import MotionArbiter
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
from vision.obstacle_detect import ObstacleResult


def build_safety() -> SafetyManager:
    return SafetyManager(SafetyConfig(30, 20, 220, 60, 35, 3, 8))


class RecordingFakeDrone(FakeDroneAdapter):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.rc_history = []

    def move_rc(self, left_right, forward_backward, up_down, yaw) -> None:
        self.rc_history.append((left_right, forward_backward, up_down, yaw))
        super().move_rc(left_right, forward_backward, up_down, yaw)


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


class ScriptedTargetDetector:
    def __init__(self, sequence) -> None:
        self.sequence = list(sequence)
        self.reads = 0

    def detect(self, frame):
        index = min(self.reads, len(self.sequence) - 1)
        self.reads += 1
        return self.sequence[index]

    def draw_debug(self, frame, result):
        return frame


class ScriptedObstacleDetector:
    """Return a fixed obstacle-observation sequence, one result per decide."""

    def __init__(self, sequence) -> None:
        self.sequence = list(sequence)
        self.reads = 0

    def detect(self, frame, target_result):
        index = min(self.reads, len(self.sequence) - 1)
        self.reads += 1
        return self.sequence[index]

    def reset(self) -> None:
        self.reads = 0


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


class FollowSessionObstaclePriorityTestCase(unittest.TestCase):
    def build_session(self, drone, obstacle_sequence, detector=None):
        safety = build_safety()
        planner = ObstacleAvoidancePlanner(
            safety_manager=safety,
            avoidance_yaw_speed=18,
            avoidance_lateral_speed=12,
            detect_confirm_frames=3,
        )
        arbiter = MotionArbiter(
            detector=ScriptedObstacleDetector(obstacle_sequence),
            planner=planner,
        )
        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector or LostTargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "control_interval": 0.01,
                "target_search": {"enabled": True},
                "occlusion_recovery": {
                    "enabled": True,
                    "initial_lock_frames": 2,
                    "initial_scan_yaw_speed": 10,
                    "occlusion_check_seconds": 5.0,
                    "occlusion_confirm_frames": 2,
                    "lateral_speed": 10,
                    "lateral_pulse_seconds": 0.5,
                    "settle_seconds": 0.5,
                    "max_lateral_pulses": 2,
                    "reacquire_frames": 2,
                },
            },
            mode_label="REID TEST",
            initial_target_lock_frames=1,
            motion_arbiter=arbiter,
        )
        session.camera = ScriptedCamera(session, len(obstacle_sequence))
        return session

    @staticmethod
    def blocked_result() -> ObstacleResult:
        return ObstacleResult(
            found=True,
            state="BLOCKED",
            bbox=(270, 150, 120, 200),
            center=(330, 250),
            side="left",
            free_space={
                "far_left": 0.2,
                "left": 0.2,
                "center": 0.0,
                "right": 0.9,
                "far_right": 0.9,
            },
            consecutive_found_frames=3,
        )

    def test_never_locked_target_does_not_detour_around_obstacle(self) -> None:
        drone = RecordingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        session = self.build_session(drone, [self.blocked_result()] * 6)

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        # 从未稳定锁定人物：即使障碍恒在，也只能原地偏航采集画面。
        self.assertEqual(len(drone.rc_history), 6)
        self.assertTrue(all(command[:3] == (0, 0, 0) for command in drone.rc_history))
        self.assertTrue(all(command[3] == 10 for command in drone.rc_history))
        # 原有会升降/分层的搜索状态机未启动。
        self.assertFalse(session.target_search.searching)

    def test_locked_target_then_matching_blocker_uses_lateral_only_peek(self) -> None:
        drone = RecordingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        clear = ObstacleResult(found=False, state="CLEAR")
        found = {
            "found": True,
            "is_predicted": False,
            "ambiguous": False,
            "center": (320, 250),
            "area_ratio": 0.046875,
            "area": 14400,
            "bbox": (280, 160, 80, 180),
        }
        lost = {
            "found": False,
            "is_predicted": False,
            "ambiguous": False,
            "center": None,
            "area_ratio": 0.0,
            "area": 0,
            "bbox": None,
        }
        targets = [found, found, lost, lost, lost, found, found]
        # First target frame is owned by INITIAL_LOCK_VERIFY and does not run the
        # obstacle detector, hence six observations serve the remaining ticks.
        obstacles = [clear, clear, self.blocked_result(), self.blocked_result(), clear]
        session = self.build_session(
            drone,
            obstacles,
            detector=ScriptedTargetDetector(targets),
        )
        session.camera = ScriptedCamera(session, len(targets))

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        self.assertEqual(len(drone.rc_history), len(targets))
        active = [command for command in drone.rc_history if any(command)]
        self.assertEqual(active, [(10, 0, 0, 0)])
        self.assertFalse(session.target_search.searching)


if __name__ == "__main__":
    unittest.main()
