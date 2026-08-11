"""Tests for the deterministic motion arbiter and local planner behavior."""

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, Optional
from unittest.mock import patch

from control.follow_control import RCCommand
from control.motion_arbiter import MotionArbiter, MotionContext
from control.obstacle_avoidance import ObstacleAvoidancePlanner
from drone.safety import SafetyManager
from vision.obstacle_detect import ObstacleResult


def build_safety() -> SafetyManager:
    return SafetyManager.from_dict({})


class StubDetector:
    def __init__(
        self,
        result: ObstacleResult,
        error: Optional[Exception] = None,
    ) -> None:
        self.result = result
        self.error = error
        self.detect_calls = 0

    def detect(self, frame: Any, target_result: Dict[str, object]) -> ObstacleResult:
        self.detect_calls += 1
        if self.error is not None:
            raise self.error
        return self.result

    def reset(self) -> None:
        self.detect_calls = 0


class MotionArbiterTestCase(unittest.TestCase):
    def test_decide_returns_planner_command_and_observation(self) -> None:
        safety = build_safety()
        result = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="right",
            free_space={"left": 0.8, "right": 0.2},
            consecutive_found_frames=3,
            confidence=0.8,
        )
        arbiter = MotionArbiter(
            detector=StubDetector(result),
            planner=ObstacleAvoidancePlanner(
                safety_manager=safety,
                avoidance_lateral_speed=12,
            ),
        )
        decision = arbiter.decide(
            desired_command=RCCommand(0, 20, 0, 0),
            frame=object(),
            context=MotionContext(mode="test", target_result={"found": True}),
        )

        self.assertEqual(decision.observation, result)
        self.assertEqual(decision.state, "AVOIDING")
        self.assertLess(decision.command.left_right, 0)
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertGreater(decision.confidence, 0.0)

    def test_sub_sampling_reuses_last_observation_between_detections(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(found=False))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
            config={"obstacle": {"detect_every_n_frames": 2}},
        )
        context = MotionContext(mode="test", target_result={"found": True})
        first = arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        second = arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)

        # 5 次决策、每 2 帧检测一次 → 检测器只跑 3 次；复用的帧返回最近一次检测结果。
        self.assertEqual(detector.detect_calls, 3)
        self.assertIs(first.observation, second.observation)

    def test_default_keeps_detecting_every_frame(self) -> None:
        safety = build_safety()
        detector = StubDetector(ObstacleResult(found=False))
        arbiter = MotionArbiter(
            detector=detector,
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
        )
        context = MotionContext(mode="test", target_result={"found": True})
        for _ in range(3):
            arbiter.decide(RCCommand(0, 20, 0, 0), frame=object(), context=context)
        self.assertEqual(detector.detect_calls, 3)

    def test_pipeline_error_returns_zero_command(self) -> None:
        safety = build_safety()
        arbiter = MotionArbiter(
            detector=StubDetector(
                ObstacleResult(),
                error=RuntimeError("sensor failure"),
            ),
            planner=ObstacleAvoidancePlanner(safety_manager=safety),
        )
        decision = arbiter.decide(
            desired_command=RCCommand(0, 20, 0, 0),
            frame=object(),
            context=MotionContext(mode="test", target_result={"found": True}),
        )

        self.assertEqual(decision.state, "FAILSAFE")
        self.assertEqual(decision.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(decision.observation.data_quality, "planner_error")

    def test_llm_friendly_jsonl_is_written_without_blocking_decision(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            safety = build_safety()
            result = ObstacleResult(
                found=True,
                state="CAUTION",
                side="left",
                frame_index=1,
            )
            arbiter = MotionArbiter(
                detector=StubDetector(result),
                planner=ObstacleAvoidancePlanner(safety_manager=safety),
                config={"obstacle": {"log_enabled": True, "log_dir": tmp, "log_every_n_frames": 1}},
            )
            decision = arbiter.decide(
                desired_command=RCCommand(0, 20, 0, 0),
                frame=object(),
                context=MotionContext(mode="test", target_result={"found": True}),
            )
            arbiter.close()

            files = list(Path(tmp).glob("*.jsonl"))
            self.assertEqual(len(files), 1)
            payload = json.loads(files[0].read_text(encoding="utf-8").splitlines()[0])
            self.assertEqual(payload["schema_version"], 1)
            self.assertEqual(payload["mode"], "test")
            self.assertEqual(payload["decision"]["state"], decision.state)
            self.assertEqual(payload["observation"]["state"], "CAUTION")


class PlannerFreeSpaceTestCase(unittest.TestCase):
    def test_chooses_free_space_direction(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            avoidance_lateral_speed=12,
        )
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="right",
                free_space={"left": 0.8, "right": 0.1},
                consecutive_found_frames=3,
            ),
        )
        self.assertEqual(decision.state, "AVOIDING")
        self.assertLess(decision.command.left_right, 0)

    def test_blocked_candidate_brakes_until_temporal_confirmation(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            detect_confirm_frames=3,
        )
        first = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                consecutive_found_frames=1,
            ),
        )
        confirmed = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                free_space={"left": 0.6, "right": 0.4},
                consecutive_found_frames=3,
            ),
        )
        self.assertEqual(first.state, "BRAKING")
        self.assertEqual(first.command.forward_backward, 0)
        self.assertIn(confirmed.state, {"AVOIDING", "SCAN"})

    def test_scan_when_all_sectors_are_blocked(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            scan_yaw_speed=8,
        )
        decision = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                free_space={"left": 0.0, "center": 0.0, "right": 0.0},
                consecutive_found_frames=3,
            ),
        )
        self.assertEqual(decision.state, "SCAN")
        self.assertEqual(decision.command.forward_backward, 0)
        self.assertNotEqual(decision.command.yaw, 0)

    def test_avoidance_timeout_requests_landing(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            max_avoidance_seconds=5.0,
            timeout_action="land",
        )
        blocked = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="center",
            free_space={"left": 0.5, "right": 0.5},
            consecutive_found_frames=3,
        )
        with patch("control.obstacle_avoidance.monotonic", side_effect=[1.0, 6.5, 7.0]):
            first = planner.plan(RCCommand(0, 20, 0, 0), blocked)
            timed_out = planner.plan(RCCommand(0, 20, 0, 0), blocked)
            still_failsafe = planner.plan(RCCommand(0, 20, 0, 0), blocked)

        self.assertEqual(first.state, "AVOIDING")
        self.assertFalse(first.requires_landing)
        self.assertEqual(timed_out.state, "FAILSAFE")
        self.assertEqual(timed_out.action, "LAND")
        self.assertTrue(timed_out.requires_landing)
        self.assertEqual(timed_out.command.forward_backward, 0)
        # 超时后只要障碍仍可见就持续 FAILSAFE，不会自行恢复前进。
        self.assertEqual(still_failsafe.state, "FAILSAFE")
        self.assertTrue(still_failsafe.requires_landing)

    def test_avoidance_timeout_hover_mode_does_not_land(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            max_avoidance_seconds=5.0,
            timeout_action="hover",
        )
        blocked = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="center",
            free_space={"left": 0.5, "right": 0.5},
            consecutive_found_frames=3,
        )
        with patch("control.obstacle_avoidance.monotonic", side_effect=[1.0, 6.5]):
            planner.plan(RCCommand(0, 20, 0, 0), blocked)
            timed_out = planner.plan(RCCommand(0, 20, 0, 0), blocked)

        self.assertEqual(timed_out.state, "FAILSAFE")
        self.assertEqual(timed_out.action, "HOVER")
        self.assertFalse(timed_out.requires_landing)
        self.assertEqual(timed_out.command.as_tuple(), (0, 0, 0, 0))

    def test_clear_after_timeout_resets_avoidance(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            max_avoidance_seconds=5.0,
            timeout_action="land",
        )
        blocked = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="center",
            free_space={"left": 0.5, "right": 0.5},
            consecutive_found_frames=3,
        )
        with patch("control.obstacle_avoidance.monotonic", side_effect=[1.0, 6.5]):
            planner.plan(RCCommand(0, 20, 0, 0), blocked)
            timed_out = planner.plan(RCCommand(0, 20, 0, 0), blocked)
        self.assertEqual(timed_out.state, "FAILSAFE")

        cleared = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False))
        self.assertEqual(cleared.state, "CLEAR")
        self.assertEqual(cleared.action, "FOLLOW")
        # 障碍再次出现时从零开始计时，不再立即 FAILSAFE。
        with patch("control.obstacle_avoidance.monotonic", return_value=2.0):
            retry = planner.plan(
                RCCommand(0, 20, 0, 0),
                ObstacleResult(
                    found=True,
                    state="BLOCKED",
                    side="center",
                    free_space={"left": 0.9, "right": 0.1},
                    consecutive_found_frames=3,
                ),
            )
        self.assertEqual(retry.state, "AVOIDING")

    def test_recovering_holds_until_clear_confirmation(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            recovery_clear_frames=3,
        )
        planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                free_space={"left": 0.5, "right": 0.5},
                consecutive_found_frames=3,
            ),
        )
        first_clear = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False))
        second_clear = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False))
        settled = planner.plan(RCCommand(0, 20, 0, 0), ObstacleResult(found=False))

        self.assertEqual(first_clear.state, "RECOVERING")
        self.assertEqual(first_clear.action, "HOLD")
        self.assertEqual(first_clear.command.forward_backward, 0)
        self.assertEqual(second_clear.state, "RECOVERING")
        self.assertEqual(settled.state, "CLEAR")
        self.assertEqual(settled.action, "FOLLOW")
        self.assertEqual(settled.command.forward_backward, 20)

    def test_scan_also_respects_avoidance_timeout(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            max_avoidance_seconds=5.0,
        )
        low_free_space = ObstacleResult(
            found=True,
            state="BLOCKED",
            side="center",
            free_space={"left": 0.1, "right": 0.1},
            consecutive_found_frames=3,
        )
        with patch("control.obstacle_avoidance.monotonic", side_effect=[1.0, 6.5]):
            scanning = planner.plan(RCCommand(0, 20, 0, 0), low_free_space)
            timed_out = planner.plan(RCCommand(0, 20, 0, 0), low_free_space)

        self.assertEqual(scanning.state, "SCAN")
        self.assertEqual(scanning.command.forward_backward, 0)
        self.assertEqual(timed_out.state, "FAILSAFE")

    def test_generic_sector_labels_split_into_left_and_right(self) -> None:
        planner = ObstacleAvoidancePlanner(
            safety_manager=build_safety(),
            avoidance_lateral_speed=12,
        )
        right_free = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                free_space={"sector_0": 0.1, "sector_1": 0.1, "sector_2": 0.9, "sector_3": 0.9},
                consecutive_found_frames=3,
            ),
        )
        self.assertEqual(right_free.action, "DETOUR_RIGHT")
        self.assertGreater(right_free.command.left_right, 0)

        planner.reset()
        left_free = planner.plan(
            RCCommand(0, 20, 0, 0),
            ObstacleResult(
                found=True,
                state="BLOCKED",
                side="center",
                free_space={"sector_0": 0.9, "sector_1": 0.9, "sector_2": 0.1, "sector_3": 0.1},
                consecutive_found_frames=3,
            ),
        )
        self.assertEqual(left_free.action, "DETOUR_LEFT")
        self.assertLess(left_free.command.left_right, 0)


if __name__ == "__main__":
    unittest.main()
