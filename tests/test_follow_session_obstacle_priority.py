"""Integration tests for obstacle-priority behavior during target loss.

目标丢失 + 障碍同时出现时，避障优先主动绕行并暂停 ReID 搜索；障碍清除后再
恢复丢失处理（重新触发找人）。这些测试直接驱动 FollowSession._loop，不依赖
OpenCV（用 numpy 假帧 + 脚本化检测器）。
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
    def build_session(self, drone, obstacle_sequence):
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
            detector=LostTargetDetector(),
            follow_controller=FollowController(safety_manager=safety),
            config={
                "display_console_camera": False,
                "control_interval": 0.01,
                "target_search": {"enabled": True},
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
            side="left",
            consecutive_found_frames=3,
            front_distance_cm=60.0,
            front_distance_status="valid",
        )

    def test_target_loss_with_blocked_obstacle_detours_and_pauses_search(self) -> None:
        drone = RecordingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        session = self.build_session(drone, [self.blocked_result()] * 6)

        with patch("control.follow_session.sleep", return_value=None):
            session._loop()

        # 目标恒丢 + 障碍恒在：始终主动绕行（前进恒为 0），而不是原地刹车。
        self.assertEqual(len(drone.rc_history), 6)
        self.assertTrue(all(command[1] == 0 for command in drone.rc_history))
        self.assertTrue(
            any(command[0] != 0 or command[3] != 0 for command in drone.rc_history)
        )
        # 障碍存在期间不调用 target_search.update，搜索被暂停。
        self.assertFalse(session.target_search.searching)

    def test_target_loss_detours_then_turns_left_after_obstacle_clears(self) -> None:
        drone = RecordingFakeDrone(verbose_rc=False)
        drone.height_cm = 70
        clear = ObstacleResult(
            found=False,
            state="CLEAR",
            front_distance_cm=80.0,
            front_distance_status="valid",
        )
        sequence = [self.blocked_result()] + [clear] * 5
        session = self.build_session(drone, sequence)

        with (
            patch("control.follow_session.sleep", return_value=None),
            patch(
                "control.obstacle_avoidance.monotonic",
                # This fixture constructs the planner at lateral speed 12, so
                # an estimated 100 cm completes after about 8.33 seconds.
                side_effect=[0.0, 2.0, 4.0, 6.0, 8.0, 9.0],
            ),
        ):
            session._loop()

        self.assertGreater(drone.rc_history[0][0], 0)  # 向右侧移
        self.assertTrue(all(command[1] == 0 for command in drone.rc_history))
        self.assertTrue(all(command[0] > 0 for command in drone.rc_history[:-1]))
        self.assertEqual(drone.rc_history[-1], (0, 0, 0, -12))
        # 搜索状态机可能在首个无障碍观测时已创建，但避障接管期间不再推进它；
        # 最终飞行指令仍由右移/左转避障独占。
        self.assertTrue(session.target_search.searching)


if __name__ == "__main__":
    unittest.main()
