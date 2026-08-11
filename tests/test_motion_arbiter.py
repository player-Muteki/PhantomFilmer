"""Tests for the deterministic motion arbiter and local planner behavior."""

import json
import tempfile
import unittest
from pathlib import Path
from typing import Any, Dict, Optional

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


if __name__ == "__main__":
    unittest.main()
